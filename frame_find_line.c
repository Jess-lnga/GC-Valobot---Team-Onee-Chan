// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#include "frame_find_line.h"

#include "frame_find_line_helpers.h"

#define LINE_EDGE_MARGIN                 20
#define LINE_MAX_POSITION_JUMP           40
#define LINE_LOST_CONFIRM_FRAMES          1
#define LINE_SMOOTH_ALPHA_NUM             3
#define LINE_SMOOTH_ALPHA_DEN             4
#define LINE_REACQUIRE_CONFIRM_FRAMES     2

typedef enum {
    LINE_STATE_LOST = 0,
    LINE_STATE_FOUND
} line_state_t;

typedef enum {
    CONTROL_POINT_UNRELIABLE = 0,
    CONTROL_POINT_UNCERTAIN,
    CONTROL_POINT_CERTAIN
} control_point_confidence_t;

static volatile int g_line_pos = 60;
static volatile int g_line_found = 0;
static volatile int g_line_measurement_valid = 0;
static volatile int g_line_lost_frames = 0;
static volatile int g_last_seen_side = 0;

static line_state_t g_line_state = LINE_STATE_LOST;
static int g_has_valid_history = 0;
static int g_last_valid_pos = 60;
static int g_filtered_pos = 60;
static int g_last_raw_pos = 60;
static int g_has_raw_history = 0;
static int g_consecutive_reacquire_frames = 0;

static int iabs_int(int x)
{
    return (x < 0) ? -x : x;
}

static int smooth_int(int old_value, int new_value)
{
    return (LINE_SMOOTH_ALPHA_NUM * new_value
          + (LINE_SMOOTH_ALPHA_DEN - LINE_SMOOTH_ALPHA_NUM) * old_value
          + LINE_SMOOTH_ALPHA_DEN / 2) / LINE_SMOOTH_ALPHA_DEN;
}

static void update_line_position_uncertain(int pos_x, int image_center_x)
{
    if (!g_has_valid_history) {
        g_has_valid_history = 1;
        g_filtered_pos = pos_x;
    } else {
        g_filtered_pos = smooth_int(g_filtered_pos, pos_x);
    }

    g_line_pos = g_filtered_pos;

    if (g_line_pos < image_center_x) {
        g_last_seen_side = -1;
    } else if (g_line_pos > image_center_x) {
        g_last_seen_side = 1;
    }
}

static void publish_line_position(int pos_x, int image_center_x)
{
    update_line_position_uncertain(pos_x, image_center_x);
    g_last_valid_pos = pos_x;
    g_line_measurement_valid = 1;
    g_line_lost_frames = 0;
}

static control_point_confidence_t take_decision_2(line_control_point_t control_point, int width, int height)
{
    const int image_center_x = height / 2;
    (void)width;

    if (!control_point.found) {
        g_line_measurement_valid = 0;
        g_consecutive_reacquire_frames = 0;
        g_line_lost_frames++;

        if (g_line_lost_frames >= LINE_LOST_CONFIRM_FRAMES) {
            g_line_state = LINE_STATE_LOST;
            g_line_found = 0;
        }

        return CONTROL_POINT_UNRELIABLE;
    }

    {
        const int candidate_pos = control_point.center_x;
        const int in_safe_zone = (candidate_pos > LINE_EDGE_MARGIN) &&
                                 (candidate_pos < (height - 1 - LINE_EDGE_MARGIN));
        const int abrupt_jump = g_has_raw_history &&
                                (iabs_int(candidate_pos - g_last_raw_pos) > LINE_MAX_POSITION_JUMP);

        g_last_raw_pos = candidate_pos;
        g_has_raw_history = 1;

        if (g_line_state == LINE_STATE_LOST) {
            if (in_safe_zone) {
                g_consecutive_reacquire_frames++;
            } else {
                g_consecutive_reacquire_frames = 0;
            }

            if (g_consecutive_reacquire_frames >= LINE_REACQUIRE_CONFIRM_FRAMES) {
                publish_line_position(candidate_pos, image_center_x);
                g_line_state = LINE_STATE_FOUND;
                g_line_found = 1;
                return CONTROL_POINT_CERTAIN;
            }

            g_line_measurement_valid = 0;
            g_line_lost_frames++;
            return CONTROL_POINT_UNCERTAIN;
        }

        g_consecutive_reacquire_frames = 0;

        if (abrupt_jump) {
            g_line_measurement_valid = 0;
            g_line_lost_frames++;

            if (g_line_lost_frames >= LINE_LOST_CONFIRM_FRAMES) {
                g_line_state = LINE_STATE_LOST;
                g_line_found = 0;
                return CONTROL_POINT_UNRELIABLE;
            }

            return CONTROL_POINT_UNCERTAIN;
        }

        publish_line_position(candidate_pos, image_center_x);
        g_line_state = LINE_STATE_FOUND;
        g_line_found = 1;
        return CONTROL_POINT_CERTAIN;
    }
}

static void draw_decided_control_point(uint16_t *frame,
                                       int width,
                                       int height,
                                       line_control_point_t control_point,
                                       control_point_confidence_t confidence)
{
    if (!control_point.found) {
        return;
    }

    if (confidence == CONTROL_POINT_CERTAIN) {
        draw_control_point(frame, width, height, g_line_pos, control_point.center_y, COLOR_PURPLE);
        return;
    }

    if (confidence == CONTROL_POINT_UNCERTAIN) {
        draw_control_point(frame, width, height, g_line_pos, control_point.center_y, COLOR_ORANGE);
        return;
    }

    draw_control_point(frame, width, height, g_line_pos, control_point.center_y, COLOR_RED);
}

uint16_t *find_line_pos(uint16_t *frame, int width, int height)
{
    if (!frame || width <= 0 || height <= 0) {
        g_line_found = 0;
        g_line_measurement_valid = 0;
        g_line_lost_frames++;
        return frame;
    }

    filter_black_pxl(frame, width, height);
    find_black_segments(frame, width, height);

    line_control_point_t control_point = sort_line(frame, width, height);

    control_point_confidence_t confidence = take_decision_2(control_point, width, height);
    draw_decided_control_point(frame, width, height, control_point, confidence);
    
    return frame;
}

int get_line_pos(void)
{
    return g_line_pos;
}

int is_line_found(void)
{
    return g_line_found;
}

int is_line_measurement_valid(void)
{
    return g_line_measurement_valid;
}

int get_line_lost_frames(void)
{
    return g_line_lost_frames;
}

int get_last_seen_side(void)
{
    return g_last_seen_side;
}
