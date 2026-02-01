#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

#include "ov7670.h"
#include "frame_analysis.h"
#include "pca9685.h"

static void core1_entry(void) {
    init_servo_ctrl();

    wake_up();
    int mode = 1;

    bool incr_angle = true;
    bool modif_angle = true;
    bool back_to_zero_angle = false;

    bool incr_dist = true;
    bool modif_dist = true;
    

    int angle = 0;
    int step_angle = 2;


    float D = 0;
    float step_d = 0.25;

    float D_max = 4;
    float D_min = 0;

    float Angle_max = 20;
    float Angle_min = -20;


    while (true) {
        //demo(mode); 
        
        for(int i = 0; i < 1; ++i){
            move(D, 0, angle*M_PI/180);
        }

        if(modif_dist){
            if(incr_dist){
                D += step_d;
                if(D > D_max){D -= 2*step_d; incr_dist = false;}
            }else{
                D -= step_d;
                if(D < D_min){D = D_min; incr_dist = true; modif_dist = false;}
            }
        }else{
            if(incr_angle){
                angle += step_angle;

                if(angle > Angle_max){angle -= 2*step_angle; incr_angle = false;}
                if((back_to_zero_angle)&&(angle > 0)){angle = 0; modif_dist = true; back_to_zero_angle = false;}

            }else{
                angle -= step_angle;
                if(angle < Angle_min){angle = Angle_min; incr_angle = true; back_to_zero_angle = true;}
            }

        }
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
        sleep_ms(1);
    }
}
