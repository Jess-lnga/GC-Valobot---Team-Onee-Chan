#include "tof.h"

#include <stdio.h>

#include "pico/stdlib.h"

#define TOF_I2C_PORT i2c0

#define TOF_XSHUT_1_PIN 18
#define TOF_XSHUT_2_PIN 19
#define TOF_XSHUT_3_PIN 20

#define TOF_ADDR_1 0x30
#define TOF_ADDR_2 0x31
#define TOF_ADDR_3 0x32

#define REG_SYSRANGE_START            0x00
#define REG_SYSTEM_INTERRUPT_CONFIG   0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR    0x0B
#define REG_RESULT_INTERRUPT_STATUS   0x13
#define REG_RESULT_RANGE_STATUS       0x14
#define REG_I2C_SLAVE_DEVICE_ADDRESS  0x8A
#define TOF_MEAN_WINDOW               20

#define TOF_DEBUG 0

#if TOF_DEBUG
#define TOF_LOG(...) printf(__VA_ARGS__)
#else
#define TOF_LOG(...)
#endif

static tof_t g_tof_1;
static tof_t g_tof_2;
static tof_t g_tof_3;

static int d_right = -1;
static int d_front = -1;
static int d_left  = -1;

static int mean_d_right = 0;
static int mean_d_front = 0;
static int mean_d_left  = 0;

static bool g_tof_initialized = false;
static bool continuous_mesure = true;

typedef struct {
    int samples[TOF_MEAN_WINDOW];
    int index;
    int count;
    int sum;
} tof_mean_filter_t;

static tof_mean_filter_t g_mean_right;
static tof_mean_filter_t g_mean_front;
static tof_mean_filter_t g_mean_left;

static void tof_mean_reset(tof_mean_filter_t *filter)
{
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0;

    for (int i = 0; i < TOF_MEAN_WINDOW; ++i) {
        filter->samples[i] = 0;
    }
}

static int tof_mean_update(tof_mean_filter_t *filter, int sample)
{
    if (sample < 0) {
        return (filter->count > 0) ? (filter->sum / filter->count) : 0;
    }

    if (filter->count < TOF_MEAN_WINDOW) {
        filter->samples[filter->index] = sample;
        filter->sum += sample;
        filter->index = (filter->index + 1) % TOF_MEAN_WINDOW;
        filter->count++;
    } else {
        filter->sum -= filter->samples[filter->index];
        filter->samples[filter->index] = sample;
        filter->sum += sample;
        filter->index = (filter->index + 1) % TOF_MEAN_WINDOW;
    }

    return filter->sum / filter->count;
}

void pca_is_active(void) {
    continuous_mesure = false;
}

void pca_is_not_active(void) {
    continuous_mesure = true;
}

static bool write8_addr(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    int r = i2c_write_blocking(i2c, addr, buf, 2, false);
    return r == 2;
}

static bool write8(tof_t *t, uint8_t reg, uint8_t val)
{
    return write8_addr(t->i2c, t->addr, reg, val);
}

static bool read8(tof_t *t, uint8_t reg, uint8_t *val)
{
    int r1 = i2c_write_blocking(t->i2c, t->addr, &reg, 1, true);
    int r2 = i2c_read_blocking(t->i2c, t->addr, val, 1, false);
    return (r1 == 1 && r2 == 1);
}

static bool read16(tof_t *t, uint8_t reg, uint16_t *val)
{
    uint8_t b[2];
    int r1 = i2c_write_blocking(t->i2c, t->addr, &reg, 1, true);
    int r2 = i2c_read_blocking(t->i2c, t->addr, b, 2, false);
    if (r1 != 1 || r2 != 2) return false;
    *val = (uint16_t)((b[0] << 8) | b[1]);
    return true;
}

bool tof_read_model_id(tof_t *t, uint8_t *id)
{
    return read8(t, 0xC0, id);
}

bool tof_read_irq_status(tof_t *t, uint8_t *st)
{
    return read8(t, REG_RESULT_INTERRUPT_STATUS, st);
}

static bool vl53l0x_basic_init(tof_t *t)
{
    uint8_t sv = 0;

    if (!write8(t, 0x88, 0x00)) return false;
    if (!write8(t, 0x80, 0x01)) return false;
    if (!write8(t, 0xFF, 0x01)) return false;
    if (!write8(t, 0x00, 0x00)) return false;

    if (!read8(t, 0x91, &sv)) return false;
    t->stop_variable = sv;

    if (!write8(t, 0x00, 0x01)) return false;
    if (!write8(t, 0xFF, 0x00)) return false;
    if (!write8(t, 0x80, 0x00)) return false;

    if (!write8(t, REG_SYSTEM_INTERRUPT_CONFIG, 0x04)) return false;
    if (!write8(t, REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;

    t->io_timeout_us = 250000;
    TOF_LOG("TOF 0x%02X stop_variable=0x%02X\n", t->addr, t->stop_variable);
    return true;
}

static bool vl53l0x_start_continuous(tof_t *t)
{
    (void)write8(t, 0x80, 0x01);
    (void)write8(t, 0xFF, 0x01);
    (void)write8(t, 0x00, 0x00);
    (void)write8(t, 0x91, t->stop_variable);
    (void)write8(t, 0x00, 0x01);
    (void)write8(t, 0xFF, 0x00);
    (void)write8(t, 0x80, 0x00);

    if (!write8(t, REG_SYSRANGE_START, 0x02)) return false;

    t->started = true;
    sleep_ms(50);
    TOF_LOG("TOF 0x%02X continuous ranging started\n", t->addr);
    return true;
}

static void tof_stop(tof_t *t)
{
    if (!t) return;
    (void)write8(t, REG_SYSRANGE_START, 0x01);
    (void)write8(t, 0xFF, 0x01);
    (void)write8(t, 0x00, 0x00);
    (void)write8(t, 0x91, 0x00);
    (void)write8(t, 0x00, 0x01);
    (void)write8(t, 0xFF, 0x00);
    t->started = false;
}

static int tof_read_mm(tof_t *t)
{
    uint8_t st = 0;

    if (!t || !t->started) return -1;

    if (!read8(t, REG_RESULT_INTERRUPT_STATUS, &st)) {
        TOF_LOG("TOF 0x%02X failed reading interrupt status\n", t->addr);
        return t->last_mm;
    }

    if ((st & 0x07) != 0) {
        uint16_t mm = 0;
        if (!read16(t, (uint8_t)(REG_RESULT_RANGE_STATUS + 10), &mm)) {
            TOF_LOG("TOF 0x%02X failed reading distance\n", t->addr);
            return t->last_mm;
        }
        (void)write8(t, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
        t->last_mm = (int)mm;
    }

    return t->last_mm;
}

static void xshut_init_pin(uint8_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

static void xshut_set(uint8_t pin, bool en)
{
    gpio_put(pin, en ? 1 : 0);
}

static bool set_address(tof_t *t, uint8_t new_addr)
{
    if (!write8(t, REG_I2C_SLAVE_DEVICE_ADDRESS, (new_addr & 0x7F))) return false;
    t->addr = (new_addr & 0x7F);
    sleep_ms(5);
    TOF_LOG("TOF assigned new address 0x%02X\n", t->addr);
    return true;
}

static bool bringup_one(tof_t *t, uint8_t xshut_pin, uint8_t new_addr)
{
    uint8_t mid = 0;

    t->xshut_pin = xshut_pin;
    t->addr = TOF_DEFAULT_ADDR;
    t->last_mm = -1;
    t->started = false;

    xshut_set(xshut_pin, true);
    sleep_ms(10);
    TOF_LOG("TOF on XSHUT pin %u enabled\n", xshut_pin);

    if (!set_address(t, new_addr)) return false;

    if (!tof_read_model_id(t, &mid)) {
        TOF_LOG("TOF 0x%02X failed reading model id\n", t->addr);
        return false;
    }

    TOF_LOG("TOF 0x%02X model_id=0x%02X\n", t->addr, mid);

    if (!vl53l0x_basic_init(t)) return false;
    if (!vl53l0x_start_continuous(t)) return false;

    TOF_LOG("TOF 0x%02X bringup complete\n", t->addr);
    return true;
}

static bool tof_init_3(tof_t *t1, tof_t *t2, tof_t *t3,
                       i2c_inst_t *i2c,
                       uint8_t xshut1, uint8_t xshut2, uint8_t xshut3,
                       uint8_t addr1, uint8_t addr2, uint8_t addr3)
{
    if (!t1 || !t2 || !t3) return false;

    t1->i2c = i2c;
    t2->i2c = i2c;
    t3->i2c = i2c;

    xshut_init_pin(xshut1);
    xshut_init_pin(xshut2);
    xshut_init_pin(xshut3);
    sleep_ms(10);

    xshut_set(xshut1, false);
    xshut_set(xshut2, false);
    xshut_set(xshut3, false);
    sleep_ms(10);

    if (!bringup_one(t1, xshut1, addr1)) return false;
    if (!bringup_one(t2, xshut2, addr2)) return false;
    if (!bringup_one(t3, xshut3, addr3)) return false;

    return true;
}

bool tof_init_all(void)
{
    TOF_LOG("Starting TOF bringup\n");
    d_right = -1;
    d_front = -1;
    d_left = -1;
    mean_d_right = 0;
    mean_d_front = 0;
    mean_d_left = 0;
    g_tof_1.last_mm = -1;
    g_tof_2.last_mm = -1;
    g_tof_3.last_mm = -1;
    tof_mean_reset(&g_mean_right);
    tof_mean_reset(&g_mean_front);
    tof_mean_reset(&g_mean_left);
    g_tof_initialized = tof_init_3(&g_tof_1,
                                   &g_tof_2,
                                   &g_tof_3,
                                   TOF_I2C_PORT,
                                   TOF_XSHUT_1_PIN,
                                   TOF_XSHUT_2_PIN,
                                   TOF_XSHUT_3_PIN,
                                   TOF_ADDR_1,
                                   TOF_ADDR_2,
                                   TOF_ADDR_3);
    if (g_tof_initialized) {
        TOF_LOG("All TOFs initialized successfully\n");
    }
    return g_tof_initialized;
}

void mes_dist_right(void)
{
    if (!g_tof_initialized) {
        d_right = -1;
        mean_d_right = 0;
        return;
    }

    d_right = tof_read_mm(&g_tof_1);
    mean_d_right = tof_mean_update(&g_mean_right, d_right);
}

void mes_dist_front(void)
{
    if (!g_tof_initialized) {
        d_front = -1;
        mean_d_front = 0;
        return;
    }

    d_front = tof_read_mm(&g_tof_2);
    mean_d_front = tof_mean_update(&g_mean_front, d_front);
}

void mes_dist_left(void)
{
    if (!g_tof_initialized) {
        d_left = -1;
        mean_d_left = 0;
        return;
    }

    d_left = tof_read_mm(&g_tof_3);
    mean_d_left = tof_mean_update(&g_mean_left, d_left);
}


void mes_all_dist(void)
{
    mes_dist_right();
    mes_dist_front();
    mes_dist_left();
}

#define OFFSET_TOF_RIGHT_MM -80
#define OFFSET_TOF_FRONT_MM -33
#define OFFSET_TOF_LEFT_MM -40

int get_dist_right(void)
{
    return d_right + OFFSET_TOF_RIGHT_MM;
}

int get_dist_front(void)
{
    return d_front + OFFSET_TOF_FRONT_MM;
}

int get_dist_left(void)
{
    return d_left + OFFSET_TOF_LEFT_MM;
}

int get_dist_mean_right(void)
{
    return mean_d_right + OFFSET_TOF_RIGHT_MM;
}

int get_dist_mean_front(void)
{
    return mean_d_front + OFFSET_TOF_FRONT_MM;
}

int get_dist_mean_left(void)
{
    return mean_d_left + OFFSET_TOF_LEFT_MM;
}

void tof_stop_all(void)
{
    tof_stop(&g_tof_1);
    tof_stop(&g_tof_2);
    tof_stop(&g_tof_3);
    g_tof_1.last_mm = -1;
    g_tof_2.last_mm = -1;
    g_tof_3.last_mm = -1;
    d_right = -1;
    d_front = -1;
    d_left = -1;
    mean_d_right = 0;
    mean_d_front = 0;
    mean_d_left = 0;
    tof_mean_reset(&g_mean_right);
    tof_mean_reset(&g_mean_front);
    tof_mean_reset(&g_mean_left);
    g_tof_initialized = false;
}
