// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

#include "ov7670.h"
#include "frame_analysis.h"
#include "pca9685.h"
#include "line_following.h"

static volatile bool g_line_following_ready = false;

static void core1_entry(void) {
    init_servo_ctrl();

    wake_up();
    sleep_ms(2000);

    int angle_deg = 0;
    int increment = 0;

    while(true){
        follow_line_step();

        /*

        if(increment < 5){
            move(2.5, 0, 8.0f * 3.14159265f / 180.0f);
            ++increment;

        }else{
            move(2.5, 0, -8.0f * 3.14159265f / 180.0f);
            ++increment;
            if(increment >= 10){
                increment = 0;
            }
        }
        */
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

    multicore_launch_core1(core1_entry);
}

int main() {
    init_all();

    static uint16_t frame[OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT];

    while (true) {
        ov7670_capture_frame(frame);
        find_line_pos(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT);
        ov7670_send_frame_usb(frame);
    
        
        sleep_ms(1);
    }
}
