// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA [Alone :( ]

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT      i2c0
#define I2C_SDA_PIN   4      // GP4
#define I2C_SCL_PIN   5      // GP5
#define PCA_ADDR      0x40   // adresse par défaut

// Registres PCA9685
#define MODE1         0x00
#define MODE2         0x01
#define PRE_SCALE     0xFE
#define LED0_ON_L     0x06   // LEDn_ON_L = 0x06 + 4*n

bool debug = false;

// Convertit des microsecondes en ticks (sur 12 bits)
static inline uint16_t to_ticks_us(int us, float us_per_tick) {
    return (uint16_t)( (us / us_per_tick) + 0.5f ); // arrondi au plus proche
}

static void pca_write8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    (void)i2c_write_blocking(I2C_PORT, PCA_ADDR, buf, 2, false);
}

static void pca_write_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {reg, on & 0xFF, on >> 8, off & 0xFF, off >> 8};
    (void)i2c_write_blocking(I2C_PORT, PCA_ADDR, buf, 5, false);
}

static uint8_t pca_read8(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(I2C_PORT, PCA_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, PCA_ADDR, &val, 1, false);
    return val;
}

/* static void pca_set_freq(float freq_hz) {
    // prescale = round(25MHz / (4096 * freq)) - 1
    float prescale_f = 25000000.0f / (4096.0f * freq_hz) - 1.0f;
    uint8_t prescale = (uint8_t)floorf(prescale_f + 0.5f);

    uint8_t oldmode = pca_read8(MODE1);
    uint8_t sleep = (oldmode & 0x7F) | 0x10;   // sleep
    pca_write8(MODE1, sleep);
    pca_write8(PRE_SCALE, prescale);
    pca_write8(MODE1, oldmode);                // wake
    sleep_ms(5);
    pca_write8(MODE1, oldmode | 0xA1);         // auto-increment + restart
    pca_write8(MODE2, 0x04);                   // OUTDRV=1 (totem-pole)
} */

static void pca_set_freq(float freq_hz) {
    // prescale = round(25MHz / (4096 * freq)) - 1
    float prescale_f = 25000000.0f / (4096.0f * freq_hz) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

    // Lire l'état actuel
    uint8_t oldmode = pca_read8(MODE1);

    // Mettre en SLEEP pour pouvoir écrire PRE_SCALE
    uint8_t sleep = (oldmode & ~0x80) | 0x10; // clear RESTART (bit7), set SLEEP (bit4)
    pca_write8(MODE1, sleep);
    sleep_ms(1);

    // Programmer le prescaler
    pca_write8(PRE_SCALE, prescale);

    // Réveiller (SLEEP=0) en restaurant oldmode sans le bit SLEEP
    uint8_t wake = oldmode & ~0x10;
    pca_write8(MODE1, wake);
    sleep_ms(5);

    // Activer AI (auto-increment) + ALLCALL si tu veux + lancer un RESTART
    uint8_t run = (wake | 0x20 | 0x01); // AI=0x20, ALLCALL=0x01
    pca_write8(MODE1, run | 0x80);      // écrire RESTART=1
    sleep_ms(1);

    // Sorties en totem-pole
    pca_write8(MODE2, 0x04);

    // DEBUG: relire et afficher
    uint8_t m1 = pca_read8(MODE1);
    uint8_t m2 = pca_read8(MODE2);
    uint8_t ps = pca_read8(PRE_SCALE);
    printf("After init: MODE1=0x%02X MODE2=0x%02X PRESCALE=0x%02X\r\n", m1, m2, ps);
}


static void i2c_scan(void) {
    printf("I2C scan:\r\n");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        uint8_t dummy = 0;
        int r = i2c_write_blocking(I2C_PORT, addr, &dummy, 1, false);
        if (r >= 0) printf(" - Found device at 0x%02X\r\n", addr);
    }
}

int main() {
    stdio_init_all();

    // Init I2C à 400 kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    i2c_scan();

    // Config PCA9685 à 50 Hz (servos)
    pca_set_freq(50.0f);

    // 50 Hz -> période = 20 000 us ; 4096 ticks par période
    // us_par_tick ≈ 20000 / 4096 ≈ 4.8828 us
    const float us_par_tick = (1000000.0f / 50.0f) / 4096.0f;

    // Position neutre (1.5 ms) sur canal 0
    uint16_t on  = 0;
    uint16_t off = to_ticks_us(1500, us_par_tick);
    pca_write_pwm(0, on, off);

    // Balayage continu 1000us ↔ 2000us sur canal 0
    while (true) {
        printf("%s\r\n", "------------------------------------------------");
        
        printf("%s\r\n", "###################################");
        i2c_scan();
        printf("%s\r\n", "###################################");
        
        printf("Sending ticks to PCA9685\r\n");
        printf("MODE1=0x%02X MODE2=0x%02X PRESCALE=0x%02X\r\n",
            pca_read8(MODE1), pca_read8(MODE2), pca_read8(PRE_SCALE));

        for (int us = 1000; us <= 2000; us += 10) {
            uint16_t ticks = to_ticks_us(us, us_par_tick);
            //printf("%u\r\n", (unsigned)ticks);

            pca_write_pwm(0, 0, to_ticks_us(us, us_par_tick));
            sleep_ms(10);
        }
        for (int us = 2000; us >= 1000; us -= 10) {
            pca_write_pwm(0, 0, to_ticks_us(us, us_par_tick));
            sleep_ms(10);
        }
        //printf("------------------------------------------------\r\n");
        printf("%s\r\n", "------------------------------------------------");

    }
}
