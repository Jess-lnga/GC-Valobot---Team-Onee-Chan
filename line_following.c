// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#include "line_following.h"

#include <stdio.h>

#include "frame_find_line.h"
#include "pca9685.h"

#define LINE_CENTER_PIXEL           60.0f

#define TRACK_D                     2.5f
#define TRACK_THETA_T               0.0f

#define SEARCH_D                    0.0f
#define SEARCH_THETA_T              0.0f
#define SEARCH_THETA_R_RAD          0.22f

#define LINE_KP                     0.003f
#define LINE_KI                     0.0002f

#define LINE_ERROR_I_MAX            250.0f
#define LINE_THETA_R_MAX_RAD        0.13963f

#define STOP_THRESHOLD              10
#define T_SHAPE_STOP_POS_THRESHOLD  80

typedef enum {
    LINE_MODE_SEARCH = 0,
    LINE_MODE_TRACK
} line_following_mode_t;

static int last_side_score = 0;
static int stop_counter = 0;

static line_following_mode_t g_mode = LINE_MODE_SEARCH;
static bool debug = false;

static float g_line_error = 0.0f;
static float g_line_error_i = 0.0f;
static float g_theta_r_cmd = 0.0f;

static float clamp_float(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static void follow_line_track_step(void)
{
    const float line_pos = (float)get_line_pos();
    const int line_found = is_line_found();

    g_line_error = line_pos - LINE_CENTER_PIXEL;
    g_line_error_i += g_line_error;
    g_line_error_i = clamp_float(g_line_error_i, -LINE_ERROR_I_MAX, LINE_ERROR_I_MAX);

    g_theta_r_cmd = LINE_KP * g_line_error + LINE_KI * g_line_error_i;
    g_theta_r_cmd = clamp_float(g_theta_r_cmd, -LINE_THETA_R_MAX_RAD, LINE_THETA_R_MAX_RAD);

    //move_step(TRACK_D, TRACK_THETA_T, g_theta_r_cmd);
    float factor = 0;
    move_step(TRACK_D, g_theta_r_cmd*factor, g_theta_r_cmd);

    if (debug) {
        printf("state=%s line_pos=%.2f err=%.2f err_i=%.2f theta_r_deg=%.2f\n",
               line_found ? "FOUND" : "LOST",
               line_pos,
               g_line_error,
               g_line_error_i,
               g_theta_r_cmd * 180.0f / 3.14159265f);
    }
}

static void follow_line_search_step(void)
{
    int side = get_last_seen_side();

    if(last_side_score + side > 0){
        side = RIGHT_SIDE;
    }
    
    if(last_side_score + side < 0){
        side = LEFT_SIDE;
    }

    g_theta_r_cmd = (float)side * SEARCH_THETA_R_RAD;
    move_step(SEARCH_D, SEARCH_THETA_T, g_theta_r_cmd);
}

void follow_line_reset(void)
{
    g_line_error = 0.0f;
    g_line_error_i = 0.0f;
    g_theta_r_cmd = 0.0f;

    last_side_score = 0;
    stop_counter = 0;
}


bool follow_line_step(void)
{
    const int line_found = is_line_found();

    if (line_found) {
        if (g_mode == LINE_MODE_SEARCH) {
            follow_line_reset();
            g_mode = LINE_MODE_TRACK;
        }

        int left_elbow = is_left_elbow_detected();
        int right_elbow = is_right_elbow_detected();
        int t_shape = is_t_shape_detected();
        
        if(left_elbow){last_side_score--;}
        if(right_elbow){last_side_score++;}

        if(t_shape){
            stop_counter++;
            int vertical_pos = get_t_shape_center_y();

            if((vertical_pos > T_SHAPE_STOP_POS_THRESHOLD)&&(stop_counter > STOP_THRESHOLD)){
                move_step(0.0f, 0.0f, 0.0f);
                return true;
            }   
        }

        follow_line_track_step();
        return false;
    }

    if (g_mode == LINE_MODE_TRACK) {
        g_mode = LINE_MODE_SEARCH;
    }

    follow_line_search_step();
    return false;
}
