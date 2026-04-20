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

#define LABYRINTHE_FRONT_THRESHOLD_MM    180
#define LABYRINTHE_LEFT_THRESHOLD_MM     220
#define LABYRINTHE_RIGHT_THRESHOLD_MM    220

#define LABYRINTHE_LEFT_TARGET_MM        120.0f
#define LABYRINTHE_RIGHT_TARGET_MM       120.0f

#define LABYRINTHE_PI_KP                   0.0045f
#define LABYRINTHE_PI_KI                   0.00018f
#define LABYRINTHE_PI_I_MAX             2500.0f
#define LABYRINTHE_THETA_R_MAX_RAD         0.035f

#define LABYRINTHE_TURN_THETA_R_RAD        0.14f
#define LABYRINTHE_TURN_90_STEPS          14

typedef enum {
    WALL_MODE_NONE = 0,
    WALL_MODE_LEFT,
    WALL_MODE_RIGHT,
    WALL_MODE_BOTH
} wall_mode_t;

typedef struct {
    int samples[LABYRINTHE_FILTER_WINDOW];
    int index;
    int count;
    int sum;
} distance_filter_t;

static float g_wall_error_i = 0.0f;

static distance_filter_t g_filter_left = {0};
static distance_filter_t g_filter_front = {0};
static distance_filter_t g_filter_right = {0};

static int g_dist_left = -1;
static int g_dist_front = -1;
static int g_dist_right = -1;

static void distance_filter_reset(distance_filter_t *filter)
{
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0;

    for (int i = 0; i < LABYRINTHE_FILTER_WINDOW; ++i) {
        filter->samples[i] = 0;
    }
}

static int distance_filter_update(distance_filter_t *filter, int new_sample)
{
    if (new_sample < 0) {
        return (filter->count > 0) ? (filter->sum / filter->count) : -1;
    }

    if (filter->count < LABYRINTHE_FILTER_WINDOW) {
        filter->samples[filter->index] = new_sample;
        filter->sum += new_sample;
        filter->index = (filter->index + 1) % LABYRINTHE_FILTER_WINDOW;
        filter->count++;
    } else {
        filter->sum -= filter->samples[filter->index];
        filter->samples[filter->index] = new_sample;
        filter->sum += new_sample;
        filter->index = (filter->index + 1) % LABYRINTHE_FILTER_WINDOW;
    }

    return filter->sum / filter->count;
}

static void update_filtered_distances(void)
{
    mes_dist_left();
    mes_dist_front();
    mes_dist_right();

    g_dist_left = distance_filter_update(&g_filter_left, get_dist_left());
    g_dist_front = distance_filter_update(&g_filter_front, get_dist_front());
    g_dist_right = distance_filter_update(&g_filter_right, get_dist_right());
}

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

static wall_mode_t detect_wall_mode(int d_left, int d_right)
{
    const bool left_detected = is_wall_detected_left(d_left);
    const bool right_detected = is_wall_detected_right(d_right);

    if (left_detected && right_detected) return WALL_MODE_BOTH;
    if (left_detected) return WALL_MODE_LEFT;
    if (right_detected) return WALL_MODE_RIGHT;
    return WALL_MODE_NONE;
}

static float compute_wall_follow_theta_r(int d_left, int d_right, wall_mode_t wall_mode)
{
    float error = 0.0f;
    float theta_r_cmd = 0.0f;

    if (wall_mode == WALL_MODE_BOTH) {
        error = (float)d_left - (float)d_right;
    } else if (wall_mode == WALL_MODE_LEFT) {
        error = (float)d_left - LABYRINTHE_LEFT_TARGET_MM;
    } else if (wall_mode == WALL_MODE_RIGHT) {
        error = LABYRINTHE_RIGHT_TARGET_MM - (float)d_right;
    } else {
        g_wall_error_i = 0.0f;
        return 0.0f;
    }

    g_wall_error_i += error;
    g_wall_error_i = clamp_float(
        g_wall_error_i,
        -LABYRINTHE_PI_I_MAX,
        LABYRINTHE_PI_I_MAX
    );

    theta_r_cmd = LABYRINTHE_PI_KP * error + LABYRINTHE_PI_KI * g_wall_error_i;
    return clamp_float(
        theta_r_cmd,
        -LABYRINTHE_THETA_R_MAX_RAD,
        LABYRINTHE_THETA_R_MAX_RAD
    );
}

static void perform_turn_90(int turn_direction)
{
    const float theta_r_cmd = (float)turn_direction * LABYRINTHE_TURN_THETA_R_RAD;

    g_wall_error_i = 0.0f;

    for (int i = 0; i < LABYRINTHE_TURN_90_STEPS; ++i) {
        move_step(0.0f, 0.0f, theta_r_cmd);
    }
}

void labyrinthe_reset(void)
{
    g_wall_error_i = 0.0f;
    g_dist_left = -1;
    g_dist_front = -1;
    g_dist_right = -1;

    distance_filter_reset(&g_filter_left);
    distance_filter_reset(&g_filter_front);
    distance_filter_reset(&g_filter_right);
}

int labyrinthe_get_dist_left(void)
{
    return g_dist_left;
}

int labyrinthe_get_dist_front(void)
{
    return g_dist_front;
}

int labyrinthe_get_dist_right(void)
{
    return g_dist_right;
}

void solve_labyrinthe(void)
{
    int turn_direction;
    wall_mode_t wall_mode;
    float theta_r_cmd;

    update_filtered_distances();

    if (is_front_blocked(g_dist_front)) {
        turn_direction = (g_dist_left >= g_dist_right) ? 1 : -1;
        perform_turn_90(turn_direction);
        return;
    }

    wall_mode = detect_wall_mode(g_dist_left, g_dist_right);
    theta_r_cmd = compute_wall_follow_theta_r(g_dist_left, g_dist_right, wall_mode);

    move_step(LABYRINTHE_FORWARD_D, LABYRINTHE_FORWARD_THETA_T, theta_r_cmd);
}

void solve_maze(void)
{
    update_filtered_distances();

    if (is_front_blocked(g_dist_front)) {
        if (g_dist_left >= g_dist_right) {
            perform_turn_90(1);
        } else {
            perform_turn_90(-1);
        }
    }
}
