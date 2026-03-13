#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

#include "ov7670.h"
#include "frame_analysis.h"
#include "pca9685.h"
#include "line_following.h"

static void core1_entry(void) {
    init_servo_ctrl();

    wake_up();
    int mode = 2;
    
    float angle_r = 1;
    float angle_t = 0;
    float D = 2.5;

    sleep_ms(2000);

    while (true){
        //demo(mode); 
        //move(D, angle_t*M_PI/180, angle_r*M_PI/180);
        //follow_line();
        //follow_line_testing();
        //follow_line_step();
        target_align_step();
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
        ov7670_capture_frame(frame);

        //line_detection_t det;
        //frame_analyze_line_rgb565(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT, &det);

        //find_line(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT, 5);
        
        //line_detection_t line_result;
        //analyze_line_and_update_state(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT, &line_result);
        
        
        target_detection_t target_result;
        find_target_and_update_state(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT, &target_result);
        
        ov7670_send_frame_usb(frame);
        
        sleep_ms(1);
    }
}
