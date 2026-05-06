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

#define T_SHAPE_EDGE_EXTENSION_MIN         5
#define DEPASSEMENT_THRESHOLD              T_SHAPE_EDGE_EXTENSION_MIN
#define T_SHAPE_MIN_AHEAD_ROWS             4

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
static volatile int g_orange_points_valid = 0;
static volatile int g_left_orange_point_x = -1;
static volatile int g_left_orange_point_y = -1;
static volatile int g_right_orange_point_x = -1;
static volatile int g_right_orange_point_y = -1;

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
    g_orange_points_valid = 0;
    g_left_orange_point_x = -1;
    g_left_orange_point_y = -1;
    g_right_orange_point_x = -1;
    g_right_orange_point_y = -1;
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

static int get_line_band_intersection_x(int line_x,
                                        int line_y,
                                        int line_slope_q8,
                                        int band_x,
                                        int band_y,
                                        int band_slope_q8);
static int get_band_y_at_x(t_shape_detection_t t_shape, int x);
static int get_t_shape_edge_extension_min(line_control_point_t control_point);
static int get_left_intersection_x(t_shape_detection_t t_shape, line_control_point_t control_point);
static int get_right_intersection_x(t_shape_detection_t t_shape, line_control_point_t control_point);
static long point_distance_sq(int x1, int y1, int x2, int y2);
static int has_valid_shape_edges(t_shape_detection_t t_shape,
                                 line_control_point_t control_point);
static int has_t_shape_candidate(t_shape_detection_t t_shape,
                                 line_control_point_t control_point,
                                 control_point_confidence_t confidence);

static int get_t_shape_edge_extension_min(line_control_point_t control_point)
{
    (void)control_point;
    return T_SHAPE_EDGE_EXTENSION_MIN;
}

static int get_line_band_intersection_x(int line_x,
                                        int line_y,
                                        int line_slope_q8,
                                        int band_x,
                                        int band_y,
                                        int band_slope_q8)
{
    const long mk = (long)line_slope_q8 * band_slope_q8;
    const long den = 65536L - mk;
    const long base_q8 = (long)line_x * 256L + (long)line_slope_q8 * (band_y - line_y);
    long x_q8;

    if (den == 0) {
        return line_x + (line_slope_q8 * (band_y - line_y) + 128) / 256;
    }

    x_q8 = (base_q8 * 65536L - mk * (long)band_x * 256L) / den;
    return (int)((x_q8 + 128) / 256);
}

static int get_band_y_at_x(t_shape_detection_t t_shape, int x)
{
    return t_shape.center_y + (t_shape.band_slope_q8 * (x - t_shape.center_x) + 128) / 256;
}

static int get_line_edge_x_at_y(int edge_x,
                                int edge_y,
                                int edge_slope_q8,
                                int y)
{
    return edge_x + (edge_slope_q8 * (y - edge_y) + 128) / 256;
}

static int get_left_intersection_x(t_shape_detection_t t_shape, line_control_point_t control_point)
{
    return get_line_band_intersection_x(control_point.start_x,
                                        control_point.center_y,
                                        control_point.start_slope_q8,
                                        t_shape.center_x,
                                        t_shape.center_y,
                                        t_shape.band_slope_q8);
}

static int get_right_intersection_x(t_shape_detection_t t_shape, line_control_point_t control_point)
{
    return get_line_band_intersection_x(control_point.end_x,
                                        control_point.center_y,
                                        control_point.end_slope_q8,
                                        t_shape.center_x,
                                        t_shape.center_y,
                                        t_shape.band_slope_q8);
}

static long point_distance_sq(int x1, int y1, int x2, int y2)
{
    const long dx = (long)x1 - x2;
    const long dy = (long)y1 - y2;

    return dx * dx + dy * dy;
}

static int has_valid_shape_edges(t_shape_detection_t t_shape,
                                 line_control_point_t control_point)
{
    return t_shape.start_x >= 0 &&
           t_shape.end_x >= t_shape.start_x &&
           t_shape.left_ext_x >= 0 &&
           t_shape.left_ext_y >= 0 &&
           t_shape.right_ext_x >= 0 &&
           t_shape.right_ext_y >= 0 &&
           control_point.start_x >= 0 &&
           control_point.end_x >= control_point.start_x;
}

static int has_t_shape_candidate(t_shape_detection_t t_shape,
                                 line_control_point_t control_point,
                                 control_point_confidence_t confidence)
{
    (void)confidence;

    if (!t_shape.found) {
        return 0;
    }

    if (!control_point.found) {
        return 0;
    }

    return has_valid_shape_edges(t_shape, control_point);
}

static uint16_t *find_line_pos_internal(uint16_t *frame, int width, int height, int detect_t)
{
    t_shape_detection_t t_shape = { false, -1, -1, 0, 0, -1, -1, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0 };

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
        const int nearest_search_row = control_point.center_y - T_SHAPE_MIN_AHEAD_ROWS;

        t_shape = detect_t_shape(frame,
                                 width,
                                 height,
                                 control_point.slope_q8,
                                 nearest_search_row);
        if (t_shape.found) {
            control_point = refine_line_slope_between(frame,
                                                      width,
                                                      height,
                                                      control_point,
                                                      t_shape.center_y);
            t_shape = detect_t_shape(frame,
                                     width,
                                     height,
                                     control_point.slope_q8,
                                     nearest_search_row);
        }
    }

    control_point_confidence_t confidence = take_decision_2(control_point, width, height);
    const int shape_candidate = detect_t &&
                                has_t_shape_candidate(t_shape,
                                                      control_point,
                                                      confidence);
    int left_overrun = 0;
    int right_overrun = 0;
    const int marker_x = project_line_center_x_at_y(control_point, t_shape.center_y);
    const int marker_y = t_shape.center_y;
    t_shape_detection_t marker_shape = t_shape;

    if (shape_candidate) {
        g_left_orange_point_x = get_left_intersection_x(t_shape, control_point);
        g_right_orange_point_x = get_right_intersection_x(t_shape, control_point);
        g_left_orange_point_y = get_band_y_at_x(t_shape, g_left_orange_point_x);
        g_right_orange_point_y = get_band_y_at_x(t_shape, g_right_orange_point_x);
        g_orange_points_valid = 1;
    }

    if (g_orange_points_valid) {
        const int threshold = get_t_shape_edge_extension_min(control_point);
        const long threshold_sq = (long)threshold * threshold;
        const long left_dist_sq =
            point_distance_sq(g_left_orange_point_x,
                              g_left_orange_point_y,
                              t_shape.left_ext_x,
                              t_shape.left_ext_y);
        const long right_dist_sq =
            point_distance_sq(g_right_orange_point_x,
                              g_right_orange_point_y,
                              t_shape.right_ext_x,
                              t_shape.right_ext_y);

        left_overrun = left_dist_sq > threshold_sq;
        right_overrun = right_dist_sq > threshold_sq;
    }

    const int valid_t_shape = left_overrun && right_overrun;
    const int elbow_shape = left_overrun != right_overrun;
    int elbow_direction = left_overrun ? LEFT_SIDE : RIGHT_SIDE;
    marker_shape.center_x = marker_x;
    marker_shape.center_y = marker_y;

    if (valid_t_shape) {
        publish_t_detection(marker_shape);
    } else if (elbow_shape) {
        publish_elbow_detection(elbow_direction, marker_x, marker_y);
    }

    draw_decided_control_point(frame, width, height, control_point, confidence);

    if (detect_t && t_shape.found) {
        draw_detection_geometry(frame, width, height, control_point, t_shape);
        {
            const int left_mid_x = (g_left_orange_point_x + t_shape.left_ext_x) / 2;
            const int left_mid_y = (g_left_orange_point_y + t_shape.left_ext_y) / 2;
            const int right_mid_x = (g_right_orange_point_x + t_shape.right_ext_x) / 2;
            const int right_mid_y = (g_right_orange_point_y + t_shape.right_ext_y) / 2;

            if (g_orange_points_valid) {
                draw_intersection_cross(frame,
                                        width,
                                        height,
                                        g_left_orange_point_x,
                                        g_left_orange_point_y,
                                        COLOR_ORANGE);
                draw_intersection_cross(frame,
                                        width,
                                        height,
                                        g_right_orange_point_x,
                                        g_right_orange_point_y,
                                        COLOR_ORANGE);
            }
            draw_intersection_cross(frame,
                                    width,
                                    height,
                                    t_shape.left_ext_x,
                                    t_shape.left_ext_y,
                                    COLOR_RED);
            draw_intersection_cross(frame,
                                    width,
                                    height,
                                    t_shape.right_ext_x,
                                    t_shape.right_ext_y,
                                    COLOR_RED);
            if (g_orange_points_valid) {
                draw_intersection_cross(frame,
                                        width,
                                        height,
                                        left_mid_x,
                                        left_mid_y,
                                        COLOR_GREEN);
                draw_intersection_cross(frame,
                                        width,
                                        height,
                                        right_mid_x,
                                        right_mid_y,
                                        COLOR_GREEN);
            }
        }
    }

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
