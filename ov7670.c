#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdio_usb.h"

#include "ov7670.h"
#include "ov7670_pio_dma.h"

// -----------------------------------------------------------------------------
// Config debug
// -----------------------------------------------------------------------------
#define USE_TEST_PATTERN 0

// -----------------------------------------------------------------------------
// Brochage
// -----------------------------------------------------------------------------
#define PIN_I2C_SDA   4
#define PIN_I2C_SCL   5

#define PIN_D0        6
#define PIN_D1        7
#define PIN_D2        8
#define PIN_D3        9
#define PIN_D4        10
#define PIN_D5        11
#define PIN_D6        12
#define PIN_D7        13

#define PIN_PCLK      14
#define PIN_VSYNC     15
#define PIN_HREF      16
#define PIN_XCLK      17

// -----------------------------------------------------------------------------
// Adresse I2C OV7670 (7 bits)
// -----------------------------------------------------------------------------
#define OV7670_ADDR  0x21

// Registres utiles
#define REG_COM7     0x12
#define REG_COM15    0x40
#define REG_TSLB     0x3A
#define REG_CLKRC    0x11
#define REG_COM3     0x0C
#define REG_COM14    0x3E
#define REG_SCALING_XSC 0x70
#define REG_SCALING_YSC 0x71
#define REG_SCALING_DCWCTR 0x72
#define REG_SCALING_PCLK_DIV 0x73
#define REG_SCALING_PCLK_DELAY 0xA2

#define REG_COM11    0x3B
#define REG_HSTART   0x17
#define REG_HSTOP    0x18
#define REG_HREF     0x32
#define REG_VSTART   0x19
#define REG_VSTOP    0x1A
#define REG_VREF     0x03

#define REG_EDGE     0x3F
#define REG_COM16    0x41
#define REG_DNSTH    0x77

// -----------------------------------------------------------------------------
// Protocole USB
// -----------------------------------------------------------------------------
static const char FRAME_MAGIC_START[4] = {'O','V','F','0'};
#define FRAME_VERSION  1

// -----------------------------------------------------------------------------
// I2C : écriture registre OV7670
// -----------------------------------------------------------------------------
static void ov7670_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c0, OV7670_ADDR, buf, 2, false);
}

// -----------------------------------------------------------------------------
// Init interne OV7670 : QQVGA 160x120 RGB565
// -----------------------------------------------------------------------------
static void ov7670_sensor_init(void) {
    ov7670_write_reg(REG_COM7, 0x80);
    sleep_ms(100);

    ov7670_write_reg(REG_CLKRC, 0x80);
    ov7670_write_reg(REG_COM11, 0x0A);

    ov7670_write_reg(REG_COM7, 0x04);

    ov7670_write_reg(REG_TSLB, 0x04);
    ov7670_write_reg(REG_COM15, 0xD0);

    ov7670_write_reg(REG_COM3, 0x04);
    ov7670_write_reg(REG_COM14, 0x1A);

    ov7670_write_reg(REG_SCALING_XSC, 0x3A);
    ov7670_write_reg(REG_SCALING_YSC, 0x35);
    ov7670_write_reg(REG_SCALING_DCWCTR, 0x22);
    ov7670_write_reg(REG_SCALING_PCLK_DIV, 0xF2);
    ov7670_write_reg(REG_SCALING_PCLK_DELAY, 0x02);

    ov7670_write_reg(REG_HSTART, 24);
    ov7670_write_reg(REG_HSTOP,  6);
    ov7670_write_reg(REG_HREF,   36);

    ov7670_write_reg(REG_VSTART, 2);
    ov7670_write_reg(REG_VSTOP,  122);
    ov7670_write_reg(REG_VREF,   0);

    ov7670_write_reg(0xB0, 0x84);
    ov7670_write_reg(0x4F, 0x80);
    ov7670_write_reg(0x50, 0x80);
    ov7670_write_reg(0x51, 0x00);
    ov7670_write_reg(0x52, 0x22);
    ov7670_write_reg(0x53, 0x5E);
    ov7670_write_reg(0x54, 0x80);
    ov7670_write_reg(0x58, 0x9E);

    ov7670_write_reg(0x13, 0xE7);
    ov7670_write_reg(0x6F, 0x9F);

    ov7670_write_reg(REG_COM16, 0x08);
    ov7670_write_reg(REG_EDGE,  0x30);
    ov7670_write_reg(REG_DNSTH, 0x00);

    sleep_ms(200);
}

// -----------------------------------------------------------------------------
// XCLK via PWM (~6 MHz)
// -----------------------------------------------------------------------------
static void xclk_init(void) {
    gpio_set_function(PIN_XCLK, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_XCLK);

    pwm_set_wrap(slice, 1);
    pwm_set_clkdiv(slice, 10.4f); // ~6 MHz (125MHz / (10.4*2))
    pwm_set_gpio_level(PIN_XCLK, 1);
    pwm_set_enabled(slice, true);
}

static void camera_pins_init(void) {
    // On les met en entrée, puis ov7670_pio_dma_init les basculera en GPIO_FUNC_PIO0
    for (int pin = PIN_D0; pin <= PIN_D7; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
    }
    gpio_init(PIN_PCLK);  gpio_set_dir(PIN_PCLK, GPIO_IN);  gpio_disable_pulls(PIN_PCLK);
    gpio_init(PIN_VSYNC); gpio_set_dir(PIN_VSYNC, GPIO_IN); gpio_disable_pulls(PIN_VSYNC);
    gpio_init(PIN_HREF);  gpio_set_dir(PIN_HREF, GPIO_IN);  gpio_disable_pulls(PIN_HREF);
}

// -----------------------------------------------------------------------------
// Test pattern optionnel
// -----------------------------------------------------------------------------
static void fill_test_pattern(uint16_t *buf) {
    for (int y = 0; y < OV7670_IMG_HEIGHT; ++y) {
        for (int x = 0; x < OV7670_IMG_WIDTH; ++x) {
            uint8_t r8 = (x * 255) / OV7670_IMG_WIDTH;
            uint8_t g8 = (y * 255) / OV7670_IMG_HEIGHT;
            uint8_t b8 = 0;

            uint16_t r = (r8 >> 3) & 0x1F;
            uint16_t g = (g8 >> 2) & 0x3F;
            uint16_t b = (b8 >> 3) & 0x1F;

            buf[y * OV7670_IMG_WIDTH + x] = (r << 11) | (g << 5) | b;
        }
    }
}

// -----------------------------------------------------------------------------
// API : init caméra complète + init PIO/DMA
// -----------------------------------------------------------------------------
void ov7670_init(void) {
    // I2C caméra à 100k pour init registres
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    xclk_init();
    camera_pins_init();
    ov7670_sensor_init();

    // Init PIO + DMA
    (void)ov7670_pio_dma_init(
        PIN_D0, PIN_PCLK, PIN_VSYNC, PIN_HREF,
        OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT
    );
}

// -----------------------------------------------------------------------------
// API : capture frame (PIO + DMA)
// -----------------------------------------------------------------------------
void ov7670_capture_frame(uint16_t *buf) {
#if USE_TEST_PATTERN
    fill_test_pattern(buf);
    return;
#else
    if (!ov7670_pio_dma_capture(buf)) {
        // en cas d'échec capture -> noir (frame "valide" côté protocole)
        for (int i = 0; i < OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT; ++i) {
            buf[i] = 0;
        }
    }
#endif
}

// -----------------------------------------------------------------------------
// API : envoi USB (binaire)
// -----------------------------------------------------------------------------
void ov7670_send_frame_usb(uint16_t *buf) {
    uint16_t width  = OV7670_IMG_WIDTH;
    uint16_t height = OV7670_IMG_HEIGHT;
    uint32_t payload_len = (uint32_t)width * (uint32_t)height * 2;

    uint32_t sum = 0;
    for (int i = 0; i < width * height; ++i) sum += buf[i];
    uint16_t checksum = (uint16_t)(sum & 0xFFFF);

    fwrite(FRAME_MAGIC_START, 1, 4, stdout);
    fwrite((uint8_t[]){FRAME_VERSION}, 1, 1, stdout);
    fwrite(&width,  1, 2, stdout);
    fwrite(&height, 1, 2, stdout);
    fwrite(&payload_len, 1, 4, stdout);
    fwrite(&checksum,    1, 2, stdout);
    fwrite(buf, 2, width * height, stdout);
    fflush(stdout);
}
