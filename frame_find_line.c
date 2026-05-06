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

#define T_SHAPE_MIN_VALID_THICKNESS       15
#define T_SHAPE_MAX_LINE_CENTER_DELTA_MIN 14
#define T_SHAPE_MAX_LINE_CENTER_DELTA_DEN  6
#define T_SHAPE_LINE_WIDTH_MARGIN          4
#define T_SHAPE_MIN_VALID_WIDTH           60
#define T_SHAPE_MIN_VALID_WIDTH_RATIO_NUM  1
#define T_SHAPE_MIN_VALID_WIDTH_RATIO_DEN  2
#define T_SHAPE_EDGE_EXTENSION_MIN         5

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
static volatile int g_t_detected = 0;
static volatile int g_t_horizontal_pos = -1;
static volatile int g_t_vertical_pos = -1;
static volatile int g_left_elbow_detected = 0;
static volatile int g_left_elbow_horizontal_pos = -1;
static volatile int g_left_elbow_vertical_pos = -1;
static volatile int g_right_elbow_detected = 0;
static volatile int g_right_elbow_horizontal_pos = -1;
static volatile int g_right_elbow_vertical_pos = -1;

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

static void clear_shape_detections(void)
{
    g_t_detected = 0;
    g_t_horizontal_pos = -1;
    g_t_vertical_pos = -1;
    g_left_elbow_detected = 0;
    g_left_elbow_horizontal_pos = -1;
    g_left_elbow_vertical_pos = -1;
    g_right_elbow_detected = 0;
    g_right_elbow_horizontal_pos = -1;
    g_right_elbow_vertical_pos = -1;
}

static void publish_t_detection(t_shape_detection_t t_shape)
{
    g_t_detected = 1;
    g_t_horizontal_pos = t_shape.center_x;
    g_t_vertical_pos = t_shape.center_y;
}

static void publish_elbow_detection(int direction, int horizontal_pos, int vertical_pos)
{
    if (direction < 0) {
        g_left_elbow_detected = 1;
        g_left_elbow_horizontal_pos = horizontal_pos;
        g_left_elbow_vertical_pos = vertical_pos;
        return;
    }

    g_right_elbow_detected = 1;
    g_right_elbow_horizontal_pos = horizontal_pos;
    g_right_elbow_vertical_pos = vertical_pos;
}

static int max_int(int a, int b)
{
    return (a > b) ? a : b;
}

static int project_line_center_x_at_y(line_control_point_t control_point, int y)
{
    const int dy = y - control_point.center_y;
    return control_point.center_x + (control_point.slope_q8 * dy + 128) / 256;
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
        g_last_seen_side = LEFT_SIDE;  //-1
    } else if (g_line_pos > image_center_x) {
        g_last_seen_side = RIGHT_SIDE; //1
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

static int get_t_shape_max_center_delta(int rotated_width);
static int is_t_shape_width_valid(t_shape_detection_t t_shape,
                                  line_control_point_t control_point,
                                  int rotated_width);
static int get_t_shape_edge_extension_min(line_control_point_t control_point);
static int get_t_shape_left_extension(t_shape_detection_t t_shape,
                                      line_control_point_t control_point);
static int get_t_shape_right_extension(t_shape_detection_t t_shape,
                                       line_control_point_t control_point);
static int has_valid_shape_edges(t_shape_detection_t t_shape,
                                 line_control_point_t control_point);
static int is_t_shape_edges_valid(t_shape_detection_t t_shape,
                                  line_control_point_t control_point);

static int is_t_shape_valid(t_shape_detection_t t_shape,
                            line_control_point_t control_point,
                            control_point_confidence_t confidence,
                            int rotated_width)
{
    if (!t_shape.found) {
        return 0;
    }

    if (!control_point.found || confidence != CONTROL_POINT_CERTAIN) {
        return 0;
    }

    if (t_shape.thickness < T_SHAPE_MIN_VALID_THICKNESS) {
        return 0;
    }

    if (!is_t_shape_width_valid(t_shape, control_point, rotated_width)) {
        return 0;
    }

    if (!is_t_shape_edges_valid(t_shape, control_point)) {
        return 0;
    }

    return 1;
}

static int get_t_shape_max_center_delta(int rotated_width)
{
    return max_int(T_SHAPE_MAX_LINE_CENTER_DELTA_MIN,
                   rotated_width / T_SHAPE_MAX_LINE_CENTER_DELTA_DEN);
}

static int is_t_shape_width_valid(t_shape_detection_t t_shape,
                                  line_control_point_t control_point,
                                  int rotated_width)
{
    const int min_absolute_width =
        max_int(T_SHAPE_MIN_VALID_WIDTH,
                (rotated_width * T_SHAPE_MIN_VALID_WIDTH_RATIO_NUM) /
                T_SHAPE_MIN_VALID_WIDTH_RATIO_DEN);

    if (control_point.width > 0 &&
        t_shape.width > control_point.width + T_SHAPE_LINE_WIDTH_MARGIN) {
        return 1;
    }

    return t_shape.width >= min_absolute_width;
}

static int get_t_shape_edge_extension_min(line_control_point_t control_point)
{
    (void)control_point;
    return T_SHAPE_EDGE_EXTENSION_MIN;
}

static int get_t_shape_left_extension(t_shape_detection_t t_shape,
                                      line_control_point_t control_point)
{
    const int dy = t_shape.center_y - control_point.center_y;
    const int line_start_at_shape_y_q8 =
        control_point.start_x * 256 + control_point.slope_q8 * dy;

    return (line_start_at_shape_y_q8 - t_shape.start_x * 256 + 128) / 256;
}

static int get_t_shape_right_extension(t_shape_detection_t t_shape,
                                       line_control_point_t control_point)
{
    const int dy = t_shape.center_y - control_point.center_y;
    const int line_end_at_shape_y_q8 =
        control_point.end_x * 256 + control_point.slope_q8 * dy;

    return (t_shape.end_x * 256 - line_end_at_shape_y_q8 + 128) / 256;
}

static int has_valid_shape_edges(t_shape_detection_t t_shape,
                                 line_control_point_t control_point)
{
    return t_shape.start_x >= 0 &&
           t_shape.end_x >= t_shape.start_x &&
           control_point.start_x >= 0 &&
           control_point.end_x >= control_point.start_x;
}

static int is_t_shape_edges_valid(t_shape_detection_t t_shape,
                                  line_control_point_t control_point)
{
    const int min_extension = get_t_shape_edge_extension_min(control_point);

    if (!has_valid_shape_edges(t_shape, control_point)) {
        return 0;
    }

    return get_t_shape_left_extension(t_shape, control_point) >= min_extension &&
           get_t_shape_right_extension(t_shape, control_point) >= min_extension;
}

static int is_elbow_shape(t_shape_detection_t t_shape,
                          line_control_point_t control_point,
                          control_point_confidence_t confidence,
                          int rotated_width)
{
    if (!t_shape.found) {
        return 0;
    }

    if (!control_point.found || confidence != CONTROL_POINT_CERTAIN) {
        return 0;
    }

    if (t_shape.thickness < T_SHAPE_MIN_VALID_THICKNESS) {
        return 0;
    }

    if (has_valid_shape_edges(t_shape, control_point)) {
        const int min_extension = get_t_shape_edge_extension_min(control_point);
        const int left_extension = get_t_shape_left_extension(t_shape, control_point);
        const int right_extension = get_t_shape_right_extension(t_shape, control_point);

        if ((left_extension >= min_extension) != (right_extension >= min_extension)) {
            return 1;
        }
    }

    return iabs_int(t_shape.center_x - control_point.center_x) >
           get_t_shape_max_center_delta(rotated_width);
}

static int get_elbow_direction(t_shape_detection_t t_shape,
                               line_control_point_t control_point)
{
    if (has_valid_shape_edges(t_shape, control_point)) {
        const int left_extension = get_t_shape_left_extension(t_shape, control_point);
        const int right_extension = get_t_shape_right_extension(t_shape, control_point);

        if (left_extension > right_extension) {
            return LEFT_SIDE;
        }

        return RIGHT_SIDE;
    }

    return (t_shape.center_x < control_point.center_x) ? LEFT_SIDE : RIGHT_SIDE;
}

static uint16_t *find_line_pos_internal(uint16_t *frame, int width, int height, int detect_t)
{
    t_shape_detection_t t_shape = { false, -1, -1, 0, 0, -1, -1, 0, 0, 0 };

    if (!frame || width <= 0 || height <= 0) {
        g_line_found = 0;
        g_line_measurement_valid = 0;
        g_line_lost_frames++;
        clear_shape_detections();
        return frame;
    }

    clear_shape_detections();

    filter_black_pxl(frame, width, height);

    find_black_segments(frame, width, height);

    line_control_point_t control_point = sort_line(frame, width, height);

    if (detect_t && control_point.found) {
        t_shape = detect_t_shape(frame, width, height, control_point.slope_q8);
    }

    control_point_confidence_t confidence = take_decision_2(control_point, width, height);
    const int valid_t_shape = detect_t &&
                              is_t_shape_valid(t_shape, control_point, confidence, height);
    const int elbow_shape = detect_t &&
                            !valid_t_shape &&
                            is_elbow_shape(t_shape, control_point, confidence, height);
    const int elbow_direction = get_elbow_direction(t_shape, control_point);
    const int marker_x = project_line_center_x_at_y(control_point, t_shape.center_y);
    const int marker_y = t_shape.center_y;
    t_shape_detection_t marker_shape = t_shape;
    marker_shape.center_x = marker_x;
    marker_shape.center_y = marker_y;

    if (valid_t_shape) {
        publish_t_detection(marker_shape);
    } else if (elbow_shape) {
        publish_elbow_detection(elbow_direction, marker_x, marker_y);
    }

    draw_decided_control_point(frame, width, height, control_point, confidence);

    if (valid_t_shape) {
        draw_t_shape_marker(frame, width, height, marker_shape, COLOR_RED);
    } else if (elbow_shape) {
        draw_elbow_marker(frame,
                          width,
                          height,
                          marker_x,
                          marker_y,
                          elbow_direction,
                          control_point.slope_q8,
                          COLOR_ORANGE);
    }
    
    return frame;
}

uint16_t *find_line_pos(uint16_t *frame, int width, int height)
{
    return find_line_pos_internal(frame, width, height, 0);
}

uint16_t *find_line_pos_and_detect_t_shape(uint16_t *frame, int width, int height)
{
    return find_line_pos_internal(frame, width, height, 1);
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

int is_t_shape_detected(void)
{
    return is_t_detected();
}

int get_t_shape_center_x(void)
{
    return get_horizontal_t_pos();
}

int get_t_shape_center_y(void)
{
    return get_vertical_t_pos();
}

int is_t_detected(void)
{
    return g_t_detected;
}

int get_horizontal_t_pos(void)
{
    return g_t_horizontal_pos;
}

int get_vertical_t_pos(void)
{
    return g_t_vertical_pos;
}

int is_left_elbow_detected(void)
{
    return g_left_elbow_detected;
}

int get_horizontal_left_elbow_pos(void)
{
    return g_left_elbow_horizontal_pos;
}

int get_vertical_left_elbow_pos(void)
{
    return g_left_elbow_vertical_pos;
}

int is_right_elbow_detected(void)
{
    return g_right_elbow_detected;
}

int get_horizontal_right_elbow_pos(void)
{
    return g_right_elbow_horizontal_pos;
}

int get_vertical_right_elbow_pos(void)
{
    return g_right_elbow_vertical_pos;
}
