#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdio_usb.h"   // pour désactiver CRLF

// -----------------------------------------------------------------------------
// Config debug
// -----------------------------------------------------------------------------
#define USE_TEST_PATTERN 0  // 1 = dégradé de test, 0 = vraie caméra

// -----------------------------------------------------------------------------
// Paramètres image
// -----------------------------------------------------------------------------
#define IMG_WIDTH   160
#define IMG_HEIGHT  120

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

#define PIN_PCLK      14   // PCLK
#define PIN_VSYNC     15   // VSYNC
#define PIN_HREF      16   // HREF
#define PIN_XCLK      17   // XCLK

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

// Netteté / débruitage
#define REG_EDGE     0x3F   // Edge enhancement
#define REG_COM16    0x41   // de-noise / AWB gain / etc.
#define REG_DNSTH    0x77   // De-noise range control

// -----------------------------------------------------------------------------
// Protocole USB
// -----------------------------------------------------------------------------
static const char FRAME_MAGIC_START[4] = {'O','V','F','0'};
#define FRAME_VERSION  1

// -----------------------------------------------------------------------------
// I2C : écriture d'un registre OV7670
// -----------------------------------------------------------------------------
static void ov7670_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c0, OV7670_ADDR, buf, 2, false);
}

// -----------------------------------------------------------------------------
// Init OV7670 : QQVGA (160x120) RGB565 propre
// -----------------------------------------------------------------------------
static void ov7670_init(void) {
    // Reset global
    ov7670_write_reg(REG_COM7, 0x80);
    sleep_ms(100);

    // Horloge interne (PLL + prescaler)
    // 0x80 = use internal PLL, auto prescale
    ov7670_write_reg(REG_CLKRC, 0x80);

    // COM11 : auto 50/60Hz + timing expo
    ov7670_write_reg(REG_COM11, 0x0A);

    // Format RGB
    ov7670_write_reg(REG_COM7, 0x04);  // RGB, base

    // RGB565
    ov7670_write_reg(REG_TSLB, 0x04);
    ov7670_write_reg(REG_COM15, 0xD0); // RGB565, full range

    // Downsampling + scaling (QQVGA 160x120)
    ov7670_write_reg(REG_COM3, 0x04);   // DCW enable
    ov7670_write_reg(REG_COM14, 0x1A);  // PCLK/4, scaling manuel

    ov7670_write_reg(REG_SCALING_XSC, 0x3A);
    ov7670_write_reg(REG_SCALING_YSC, 0x35);
    ov7670_write_reg(REG_SCALING_DCWCTR, 0x22);   // 4x downsample (QQVGA)
    ov7670_write_reg(REG_SCALING_PCLK_DIV, 0xF2);
    ov7670_write_reg(REG_SCALING_PCLK_DELAY, 0x02);

    // Fenêtre (crop) qui marchait déjà chez toi
    ov7670_write_reg(REG_HSTART, 24);
    ov7670_write_reg(REG_HSTOP,  6);
    ov7670_write_reg(REG_HREF,   36);

    ov7670_write_reg(REG_VSTART, 2);
    ov7670_write_reg(REG_VSTOP,  122);
    ov7670_write_reg(REG_VREF,   0);

    // Réglages couleur de base
    ov7670_write_reg(0xB0, 0x84);
    ov7670_write_reg(0x4F, 0x80);
    ov7670_write_reg(0x50, 0x80);
    ov7670_write_reg(0x51, 0x00);
    ov7670_write_reg(0x52, 0x22);
    ov7670_write_reg(0x53, 0x5E);
    ov7670_write_reg(0x54, 0x80);
    ov7670_write_reg(0x58, 0x9E);

    // AWB / AGC / AEC (auto)
    ov7670_write_reg(0x13, 0xE7);  // AGC, AEC, AWB ON
    ov7670_write_reg(0x6F, 0x9F);

    // --- Netteté / bruit (compromis) ---
    // COM16 :
    //  bit3 = AWB gain enable
    //  bit4 = denoise auto enable (on le laisse à 0 pour éviter le "huileux" fort)
    ov7670_write_reg(REG_COM16, 0x08);  // AWB gain, denoise auto OFF

    // EDGE : renforcement des contours (modéré)
    ov7670_write_reg(REG_EDGE,  0x30);  // tu peux tester 0x20..0x3C

    // DNSTH : seuil de débruitage (0x00 = peu de lissage)
    ov7670_write_reg(REG_DNSTH, 0x00);

    sleep_ms(200);
}

// -----------------------------------------------------------------------------
// Génération de XCLK via PWM (~6 MHz, déjà testé OK chez toi)
// -----------------------------------------------------------------------------
static void xclk_init(void) {
    gpio_set_function(PIN_XCLK, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_XCLK);

    pwm_set_wrap(slice, 1);
    pwm_set_clkdiv(slice, 10.4f);       // 125 MHz / (10.4 * 2) ≈ 6 MHz
    pwm_set_gpio_level(PIN_XCLK, 1);    // 50% duty
    pwm_set_enabled(slice, true);
}

// -----------------------------------------------------------------------------
// Init des GPIO
// -----------------------------------------------------------------------------
static void camera_pins_init(void) {
    for (int pin = PIN_D0; pin <= PIN_D7; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
    }

    gpio_init(PIN_PCLK);
    gpio_set_dir(PIN_PCLK, GPIO_IN);
    gpio_disable_pulls(PIN_PCLK);

    gpio_init(PIN_VSYNC);
    gpio_set_dir(PIN_VSYNC, GPIO_IN);
    gpio_disable_pulls(PIN_VSYNC);

    gpio_init(PIN_HREF);
    gpio_set_dir(PIN_HREF, GPIO_IN);
    gpio_disable_pulls(PIN_HREF);
}

// Lecture rapide du bus D0..D7
static inline uint8_t read_data_bus(void) {
    uint32_t all = gpio_get_all();
    return (uint8_t)((all >> PIN_D0) & 0xFF);
}

// -----------------------------------------------------------------------------
// Motif de test
// -----------------------------------------------------------------------------
static void fill_test_pattern(uint16_t *buf) {
    for (int y = 0; y < IMG_HEIGHT; ++y) {
        for (int x = 0; x < IMG_WIDTH; ++x) {
            uint8_t r8 = (x * 255) / IMG_WIDTH;
            uint8_t g8 = (y * 255) / IMG_HEIGHT;
            uint8_t b8 = 0;

            uint16_t r = (r8 >> 3) & 0x1F;
            uint16_t g = (g8 >> 2) & 0x3F;
            uint16_t b = (b8 >> 3) & 0x1F;

            uint16_t pixel = (r << 11) | (g << 5) | b;
            buf[y * IMG_WIDTH + x] = pixel;
        }
    }
}

// -----------------------------------------------------------------------------
// Capture d'une frame RGB565 160x120
// -----------------------------------------------------------------------------
static void capture_frame(uint16_t *buf) {
    const int max_pixels = IMG_WIDTH * IMG_HEIGHT;
    int pixel_count = 0;

    // 1) Attendre un front montant de VSYNC
    int prev_vsync = gpio_get(PIN_VSYNC);
    while (true) {
        int v = gpio_get(PIN_VSYNC);
        if (!prev_vsync && v) {
            break;
        }
        prev_vsync = v;
        tight_loop_contents();
    }

    // 2) Attendre que VSYNC retombe à 0
    while (gpio_get(PIN_VSYNC)) {
        tight_loop_contents();
    }

    // 3) Lire lignes tant que VSYNC bas
    while (!gpio_get(PIN_VSYNC) && pixel_count < max_pixels) {
        // Attendre début de ligne
        while (!gpio_get(PIN_HREF)) {
            if (gpio_get(PIN_VSYNC)) {
                goto end;
            }
            tight_loop_contents();
        }

        // 4) Lire pixels tant que HREF haut
        int col = 0;
        while (gpio_get(PIN_HREF) && !gpio_get(PIN_VSYNC) && pixel_count < max_pixels) {
            // Octet de poids fort
            while (!gpio_get(PIN_PCLK)) tight_loop_contents();
            uint8_t hi = read_data_bus();
            while (gpio_get(PIN_PCLK)) tight_loop_contents();

            // Octet de poids faible
            while (!gpio_get(PIN_PCLK)) tight_loop_contents();
            uint8_t lo = read_data_bus();
            while (gpio_get(PIN_PCLK)) tight_loop_contents();

            if (col < IMG_WIDTH) {
                uint16_t pixel = ((uint16_t)hi << 8) | lo;
                buf[pixel_count++] = pixel;
            }
            col++;
        }

        // Si la ligne envoyée par la caméra était plus large,
        // on ignore le surplus (col > IMG_WIDTH) mais on garde la synchro.
    }

end:
    // Complète en noir si jamais on a eu moins de pixels
    for (int i = pixel_count; i < max_pixels; ++i) {
        buf[i] = 0;
    }
}

// -----------------------------------------------------------------------------
// Envoi USB
// -----------------------------------------------------------------------------
static void send_frame_usb(uint16_t *buf) {
    uint16_t width = IMG_WIDTH;
    uint16_t height = IMG_HEIGHT;
    uint32_t payload_len = (uint32_t)IMG_WIDTH * (uint32_t)IMG_HEIGHT * 2;

    uint32_t sum = 0;
    for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; ++i) {
        sum += buf[i];
    }
    uint16_t checksum = (uint16_t)(sum & 0xFFFF);

    fwrite(FRAME_MAGIC_START, 1, 4, stdout);
    fwrite((uint8_t[]){FRAME_VERSION}, 1, 1, stdout);
    fwrite(&width, 1, 2, stdout);
    fwrite(&height, 1, 2, stdout);
    fwrite(&payload_len, 1, 4, stdout);
    fwrite(&checksum, 1, 2, stdout);

    fwrite(buf, 2, IMG_WIDTH * IMG_HEIGHT, stdout);

    fflush(stdout);
}

// -----------------------------------------------------------------------------
// main()
// -----------------------------------------------------------------------------
int main() {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    setvbuf(stdout, NULL, _IONBF, 0);

    sleep_ms(2000);

    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    xclk_init();
    camera_pins_init();
    ov7670_init();

    static uint16_t frame[IMG_WIDTH * IMG_HEIGHT];

    while (1) {
#if USE_TEST_PATTERN
        fill_test_pattern(frame);
#else
        capture_frame(frame);
#endif
        send_frame_usb(frame);
        // pas de sleep ici : laisse la caméra dicter le FPS
    }

    return 0;
}
