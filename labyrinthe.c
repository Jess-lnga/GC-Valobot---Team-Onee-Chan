// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jerome ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jerome ESSOLA ELANGA

#include "labyrinthe.h"

#include <math.h>

#include "pca9685.h"
#include "tof.h"

#define LABYRINTHE_FORWARD_D               2.5f
#define LABYRINTHE_FORWARD_THETA_T         0.0f
#define LABYRINTHE_FILTER_WINDOW             5

#define LABYRINTHE_FRONT_THRESHOLD_MM    130
#define LABYRINTHE_LEFT_THRESHOLD_MM     140
#define LABYRINTHE_RIGHT_THRESHOLD_MM    140

#define LABYRINTHE_LEFT_TARGET_MM        120.0f
#define LABYRINTHE_RIGHT_TARGET_MM       120.0f

#define LABYRINTHE_PI_KP                   0.008f
#define LABYRINTHE_PI_KI                   0.00018f
#define LABYRINTHE_PI_I_MAX             2500.0f
#define LABYRINTHE_THETA_R_MAX_RAD         0.2f

#define LABYRINTHE_TURN_THETA_R_RAD        0.28f //0.14f
#define LABYRINTHE_TURN_90_STEPS           7 //14

#define TURN_LEFT  -1
#define TURN_RIGHT 1

static int hard_coded_path[5] = {TURN_RIGHT, TURN_RIGHT, TURN_LEFT, TURN_LEFT, TURN_RIGHT};
static int path_index = 0;


static float clamp_float(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static bool is_wall_detected_left(int dist_mm)
{
    return dist_mm > 0 && dist_mm <= LABYRINTHE_LEFT_THRESHOLD_MM;
}

static bool is_wall_detected_right(int dist_mm)
{
    return dist_mm > 0 && dist_mm <= LABYRINTHE_RIGHT_THRESHOLD_MM;
}

static bool is_front_blocked(int dist_mm)
{
    return dist_mm > 0 && dist_mm <= LABYRINTHE_FRONT_THRESHOLD_MM;
}

static void perform_turn_90(int turn_direction)
{
    if(turn_direction != hard_coded_path[path_index]){
        return; // The turn direction is not the one we expect, so we will try again 
    }

    const float theta_r_cmd = (float)turn_direction * LABYRINTHE_TURN_THETA_R_RAD;
    
    /*
    for (int i = 0; i < LABYRINTHE_TURN_90_STEPS; ++i) {
        move_step(0.0f, 0.0f, theta_r_cmd);
        mes_all_dist();
    }
    */

    mes_all_dist();

    int d_front = get_dist_mean_front();
    int d_left  = get_dist_mean_left();
    int d_right = get_dist_mean_right();
    
    int epsilon_front_mm = 220;
    int epsilon_side_mm = 0;

    int INT_MAX = 2147483647;

    if(turn_direction == -1){
        d_front = get_dist_mean_front();
        d_right = get_dist_mean_right();

        //while((abs(d_right - front_dist_mm) > 20)||!is_front_blocked(d_front - epsilon_mm)){
        while(is_front_blocked(clamp_float(d_front - epsilon_front_mm, 1, INT_MAX)) || !is_wall_detected_right(d_right + epsilon_side_mm)){
            move_step(0.0f, 0.0f, theta_r_cmd);
            
            
            mes_all_dist();
            d_right = get_dist_mean_right();
            d_front = get_dist_mean_front();
        }

    }else{
        d_front = get_dist_mean_front();
        d_left = get_dist_mean_left();
        
        while(is_front_blocked(clamp_float(d_front - epsilon_front_mm, 1, INT_MAX)) || !is_wall_detected_left(d_left + epsilon_side_mm)){
            move_step(0.0f, 0.0f, theta_r_cmd);
            
            
            mes_all_dist();
            d_left = get_dist_mean_left();
            d_front = get_dist_mean_front();
        }

    }

    ++path_index;
}


bool solve_maze(void){

    if(path_index >= 5){
        return true; //The maze should be solved

    }else{
        int d_front = get_dist_mean_front();
        int d_left  = get_dist_mean_left();
        int d_right = get_dist_mean_right();

        

        if (is_front_blocked(d_front)){
            if (d_left >= d_right) {
                perform_turn_90(TURN_LEFT);
            } else {
                perform_turn_90(TURN_RIGHT);
            }
        }else{
            bool left_wall = is_wall_detected_left(d_left);
            bool right_wall = is_wall_detected_right(d_right);
            float factor = 3.0f;

            if(left_wall && !right_wall){
                float error_mm = (float)(d_left - LABYRINTHE_LEFT_TARGET_MM);
                float theta_t_cmd = clamp_float(LABYRINTHE_PI_KP * error_mm, -LABYRINTHE_THETA_R_MAX_RAD, LABYRINTHE_THETA_R_MAX_RAD);
                move_step(LABYRINTHE_FORWARD_D, -theta_t_cmd, -theta_t_cmd/factor);
            
            }else if(!left_wall && right_wall){
                float error_mm = (float)(d_right - LABYRINTHE_RIGHT_TARGET_MM);
                float theta_t_cmd = clamp_float(LABYRINTHE_PI_KP * error_mm, -LABYRINTHE_THETA_R_MAX_RAD, LABYRINTHE_THETA_R_MAX_RAD);
                move_step(LABYRINTHE_FORWARD_D, theta_t_cmd, theta_t_cmd/factor);
        
            }else if(left_wall && right_wall){
                float error_mm_left = (float)(d_left - LABYRINTHE_LEFT_TARGET_MM);
                float error_mm_right = (float)(d_right - LABYRINTHE_RIGHT_TARGET_MM);
                float theta_t_cmd_left = clamp_float(LABYRINTHE_PI_KP * error_mm_left, -LABYRINTHE_THETA_R_MAX_RAD, LABYRINTHE_THETA_R_MAX_RAD);
                float theta_t_cmd_right = clamp_float(LABYRINTHE_PI_KP * error_mm_right, -LABYRINTHE_THETA_R_MAX_RAD, LABYRINTHE_THETA_R_MAX_RAD);
                
                move_step(LABYRINTHE_FORWARD_D, (theta_t_cmd_right - theta_t_cmd_left), (theta_t_cmd_right - theta_t_cmd_left) / factor);
                /*
                if(d_left < d_right){
                    float error_mm = (float)(d_left - LABYRINTHE_LEFT_TARGET_MM);
                    float theta_t_cmd = clamp_float(LABYRINTHE_PI_KP * error_mm, -LABYRINTHE_THETA_R_MAX_RAD, LABYRINTHE_THETA_R_MAX_RAD);
                    move_step(LABYRINTHE_FORWARD_D, -theta_t_cmd, -theta_t_cmd/factor);
                }else{
                    float error_mm = (float)(d_right - LABYRINTHE_RIGHT_TARGET_MM);
                    float theta_t_cmd = clamp_float(LABYRINTHE_PI_KP * error_mm, -LABYRINTHE_THETA_R_MAX_RAD, LABYRINTHE_THETA_R_MAX_RAD);
                    move_step(LABYRINTHE_FORWARD_D, theta_t_cmd, theta_t_cmd/factor);
                }
                */

                //float error_mm = (float)(d_right - d_left);
                //float theta_t_cmd = clamp_float(LABYRINTHE_PI_KP * error_mm, -LABYRINTHE_THETA_R_MAX_RAD, LABYRINTHE_THETA_R_MAX_RAD);
                //move_step(LABYRINTHE_FORWARD_D, theta_t_cmd, theta_t_cmd/factor);
            
            }else{
                move_step(LABYRINTHE_FORWARD_D, 0.0f, 0.0f);
            }
        }
    }
    return false;
}






/*
printf(
        "\033[HTOF mean | R=%4d mm | F=%4d mm | L=%4d mm        \n",
        d_right,
        d_front,
        d_left
    );
fflush(stdout);

sleep_ms(10);
*/