#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

#include "ov7670.h"
#include "frame_analysis.h"
#include "pca9685.h"

static void core1_entry(void) {
    init_servo_ctrl();   // DOIT être silencieux (pas de printf)

    int mode = 6;
    while (true) {
        demo(mode);      // DOIT être silencieux (pas de printf)
    }
}

static void init_all(void) {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(2000);

    ov7670_init();

    // ok : on repasse i2c0 à 400k après init caméra
    i2c_init(i2c0, 400 * 1000);

    //init_servo_ctrl();   // DOIT être silencieux (pas de printf)

    multicore_launch_core1(core1_entry);
}

int main() {
    init_all();

    static uint16_t frame[OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT];

    while (true) {
        //printf("I am here 0");
        ov7670_capture_frame(frame);

        // Optionnel : analyse
        line_detection_t det;
        frame_analyze_line_rgb565(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT, &det);

        ov7670_send_frame_usb(frame);
        //printf("I am here 1");

        // Optionnel : petite pause si tu veux limiter le débit
        // sleep_ms(1);
    }
}
