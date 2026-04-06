#include "line_following.h"

#include <math.h>

#include "frame_analysis.h"
#include "pca9685.h"

#define LINE_CENTER_PIXEL         60.0f

#define LINE_FOLLOW_D             2.5f
#define LINE_FOLLOW_THETA_T       0.0f

#define LINE_KP                   0.018f
#define LINE_KI                   0.0012f

#define LINE_ERROR_I_MAX         250.0f
#define LINE_THETA_R_MAX_RAD       0.45f

static float g_line_error = 0.0f;
static float g_line_error_i = 0.0f;
static float g_theta_r_cmd = 0.0f;

static float clamp_float(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

void follow_line_reset(void)
{
    g_line_error = 0.0f;
    g_line_error_i = 0.0f;
    g_theta_r_cmd = 0.0f;
}

void follow_line_step(void)
{
    const float line_pos = (float)get_line_pos();

    g_line_error = line_pos - LINE_CENTER_PIXEL;

    g_line_error_i += g_line_error;
    g_line_error_i = clamp_float(g_line_error_i, -LINE_ERROR_I_MAX, LINE_ERROR_I_MAX);

    g_theta_r_cmd = LINE_KP * g_line_error + LINE_KI * g_line_error_i;
    g_theta_r_cmd = clamp_float(g_theta_r_cmd, -LINE_THETA_R_MAX_RAD, LINE_THETA_R_MAX_RAD);

    move(LINE_FOLLOW_D, LINE_FOLLOW_THETA_T, g_theta_r_cmd);
}
