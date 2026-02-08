#include "line_following.h"

#define CENTER 60.0f


static float error;
static float rotation_angle_us = 1500;
static float k_p = 0.01; 

void follow_line_testing(){
    float line_pos = get_line_pos();
    error = line_pos - CENTER;

    rotation_angle_us += k_p * error;

    if(rotation_angle_us < 1200){ rotation_angle_us = 1200;}
    if(rotation_angle_us > 1800){ rotation_angle_us = 1800;}


    turn_without_moving((int)rotation_angle_us);

    if((rotation_angle_us == 1200)||(rotation_angle_us == 1800)){
        recenter(rotation_angle_us);
        rotation_angle_us = 1500;
    }

    
    
    //printf("Line params: LINE_POS = %.2f ERROR = %.2f ROTATION US = %.2f\r\n", line_pos, error, rotation_angle_us);
    //printf("%d\r\n", rotation_angle_us);
}

static float D = 4;
static float angle = 0;
static float K_P = 0.3;

#define ABS_ANGLE_MAX 30


void follow_line(){
    float line_pos = get_line_pos();
    error = line_pos - CENTER;

    angle = K_P * error;
    if(angle > ABS_ANGLE_MAX){ angle = ABS_ANGLE_MAX;}
    if(angle < - ABS_ANGLE_MAX){ angle = - ABS_ANGLE_MAX;}

    //printf("Line params: LINE_POS = %.2f ERROR = %.2f ROTATION ANGLE = %.2f\r\n", line_pos, error, angle);
    

    move(D, 0, angle*M_PI/180.0);
}