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

#define LINE_ERROR_I_MAX          250.0f
#define LINE_THETA_R_MAX_RAD        0.13963f

typedef enum {
    LINE_MODE_SEARCH = 0,
    LINE_MODE_TRACK
} line_following_mode_t;

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
    float factor = 1;
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
    if (side == 0) {
        side = 1;
    }

    g_theta_r_cmd = (float)side * SEARCH_THETA_R_RAD;
    //move(SEARCH_D, SEARCH_THETA_T, g_theta_r_cmd);
    move_step(SEARCH_D, SEARCH_THETA_T, g_theta_r_cmd);
}

void follow_line_reset(void)
{
    g_line_error = 0.0f;
    g_line_error_i = 0.0f;
    g_theta_r_cmd = 0.0f;
}

void follow_line_step(void)
{
    const int line_found = is_line_found();

    if (line_found) {
        if (g_mode == LINE_MODE_SEARCH) {
            follow_line_reset();
            g_mode = LINE_MODE_TRACK;
        }

        follow_line_track_step();
        return;
    }

    if (g_mode == LINE_MODE_TRACK) {
        g_mode = LINE_MODE_SEARCH;
    }

    follow_line_search_step();
}
