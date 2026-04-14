// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA

/*
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

#include "ov7670.h"
#include "frame_analysis.h"
#include "pca9685.h"
#include "line_following.h"
#include "tof.h"

static volatile bool g_line_following_ready = false;
static volatile bool g_tof_ready = false;

static void core1_entry(void) {
    init_servo_ctrl();

    wake_up();
    sleep_ms(2000);

    int angle_deg = 0;
    int increment = 0;

    while(true){
        follow_line_step();
        
        //if (!g_tof_ready) {
        //    printf("TOF init failed\n");
        //    sleep_ms(500);
        //    continue;
        //}

        //int d1 = get_dist_1();
        //int d2 = get_dist_2();
        //int d3 = get_dist_3();

        //printf("\033[H\033[J");
        //printf("TOF distances: d1=%d mm | d2=%d mm | d3=%d mm\n", d1, d2, d3);
        //printf("\033[1GTOF distances: d1=%d mm | d2=%d mm | d3=%d mm    ", d1, d2, d3);
        //fflush(stdout);




        //sleep_ms(100);
        
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
    g_tof_ready = tof_init_all();
    printf("TOF init: %s\n", g_tof_ready ? "OK" : "FAILED");

    sleep_ms(100);

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

*/