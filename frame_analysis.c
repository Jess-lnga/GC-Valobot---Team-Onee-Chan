#include "frame_analysis.h"

#include "frame_analysis_helpers.h"

#define LINE_EDGE_MARGIN                 10
#define LINE_MAX_POSITION_JUMP           20
#define LINE_FOUND_CONFIRM_FRAMES         2
#define LINE_LOST_CONFIRM_FRAMES          3
#define LINE_UNCERTAIN_MAX_FRAMES         2
#define LINE_SMOOTH_ALPHA_NUM             3
#define LINE_SMOOTH_ALPHA_DEN             4

typedef enum {
    LINE_STATE_LOST = 0,
    LINE_STATE_UNCERTAIN,
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
static int g_consecutive_found_frames = 0;
static int g_consecutive_uncertain_frames = 0;

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

static void publish_line_position(int pos_x, int image_center_x)
{
    if (!g_has_valid_history) {
        g_has_valid_history = 1;
        g_filtered_pos = pos_x;
    } else {
        g_filtered_pos = smooth_int(g_filtered_pos, pos_x);
    }

    g_last_valid_pos = pos_x;
    g_line_pos = g_filtered_pos;
    g_line_measurement_valid = 1;
    g_line_lost_frames = 0;

    if (g_line_pos < image_center_x) {
        g_last_seen_side = -1;
    } else if (g_line_pos > image_center_x) {
        g_last_seen_side = 1;
    }
}

static control_point_confidence_t take_decision(line_control_point_t control_point, int width, int height)
{
    const int image_center_x = height / 2;
    const int candidate_pos = control_point.center_x;
    const int near_border = control_point.found &&
                            ((candidate_pos <= LINE_EDGE_MARGIN) ||
                             (candidate_pos >= (height - 1 - LINE_EDGE_MARGIN)));
    const int jump_too_large = control_point.found &&
                               g_has_valid_history &&
                               (iabs_int(candidate_pos - g_last_valid_pos) > LINE_MAX_POSITION_JUMP);
    const int valid_candidate = control_point.found && !near_border && !jump_too_large;
    const int uncertain_candidate = control_point.found && !valid_candidate;
    (void)width;

    if (valid_candidate) {
        g_consecutive_found_frames++;
        g_consecutive_uncertain_frames = 0;

        publish_line_position(candidate_pos, image_center_x);

        if (g_consecutive_found_frames >= LINE_FOUND_CONFIRM_FRAMES) {
            g_line_state = LINE_STATE_FOUND;
            g_line_found = 1;
            return CONTROL_POINT_CERTAIN;
        } else if (g_line_state == LINE_STATE_LOST) {
            g_line_state = LINE_STATE_UNCERTAIN;
            g_line_found = 0;
            return CONTROL_POINT_UNCERTAIN;
        }

        return CONTROL_POINT_UNCERTAIN;
    }

    g_line_measurement_valid = 0;
    g_consecutive_found_frames = 0;

    if (uncertain_candidate) {
        g_consecutive_uncertain_frames++;
        g_line_state = LINE_STATE_UNCERTAIN;

        if (g_consecutive_uncertain_frames > LINE_UNCERTAIN_MAX_FRAMES) {
            g_line_found = 0;
            g_line_lost_frames++;
        }

        return CONTROL_POINT_UNCERTAIN;
    }

    g_consecutive_uncertain_frames = 0;
    g_line_lost_frames++;

    if (g_line_lost_frames >= LINE_LOST_CONFIRM_FRAMES) {
        g_line_state = LINE_STATE_LOST;
        g_line_found = 0;
    } else if (g_line_state == LINE_STATE_FOUND) {
        g_line_state = LINE_STATE_UNCERTAIN;
    }

    return CONTROL_POINT_UNRELIABLE;
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
        draw_control_point(frame, width, height, control_point.center_x, control_point.center_y, COLOR_ORANGE);
        return;
    }

    draw_control_point(frame, width, height, control_point.center_x, control_point.center_y, COLOR_RED);
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
    control_point_confidence_t confidence = take_decision(control_point, width, height);
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
