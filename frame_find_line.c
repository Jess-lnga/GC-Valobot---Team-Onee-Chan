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
#define SHAPE2_MIN_SIDE_EXTENSION          8
#define SHAPE2_MIN_BAND_ROWS               2
#define SHAPE2_MAX_BAND_GAP                1
#define SHAPE2_BAND_FIT_MARGIN             2
#define SHAPE2_MIN_BAND_COLUMNS            8

typedef enum {
    LINE_STATE_LOST = 0,
    LINE_STATE_FOUND
} line_state_t;

typedef enum {
    CONTROL_POINT_UNRELIABLE = 0,
    CONTROL_POINT_UNCERTAIN,
    CONTROL_POINT_CERTAIN
} control_point_confidence_t;

typedef struct {
    int found;
    int near_y;
    int far_y;
    int min_x;
    int max_x;
    int row_count;
} shape2_band_run_t;

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
static int get_line_band_segment_intersection(line_control_point_t control_point,
                                              int use_left_edge,
                                              t_shape_detection_t t_shape,
                                              int *intersection_x,
                                              int *intersection_y);
static int get_band_y_at_x(t_shape_detection_t t_shape, int x);
static int get_t_shape_edge_extension_min(line_control_point_t control_point);
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

static int imin_int(int a, int b)
{
    return (a < b) ? a : b;
}

static int imax_int(int a, int b)
{
    return (a > b) ? a : b;
}

static int clamp_int_local(int x, int xmin, int xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static inline uint16_t get_px_rotm90_local(const uint16_t *frame,
                                           int width,
                                           int height,
                                           int xv,
                                           int yv)
{
    const int xo = yv;
    const int yo = (height - 1) - xv;
    return frame[yo * width + xo];
}

static int is_track_pixel(uint16_t px)
{
    return px == COLOR_BLACK || px == COLOR_BLUE || px == COLOR_GREEN;
}

static int fit_y_from_x_q8_local(long sum_x,
                                 long sum_y,
                                 long sum_xx,
                                 long sum_xy,
                                 int count)
{
    const long den = (long)count * sum_xx - sum_x * sum_x;

    if (count < 2 || den == 0) {
        return 0;
    }

    {
        const long num = (long)count * sum_xy - sum_x * sum_y;
        return clamp_int_local((int)((num * 256 + den / 2) / den), -1024, 1024);
    }
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

static int get_line_band_segment_intersection(line_control_point_t control_point,
                                              int use_left_edge,
                                              t_shape_detection_t t_shape,
                                              int *intersection_x,
                                              int *intersection_y)
{
    const int line_x = use_left_edge ? control_point.start_x : control_point.end_x;
    const int line_slope_q8 = use_left_edge ? control_point.start_slope_q8
                                            : control_point.end_slope_q8;
    const long mk = (long)line_slope_q8 * t_shape.band_slope_q8;
    const long den = 65536L - mk;
    const int margin = imax_int(imax_int(control_point.width, t_shape.thickness), 4);
    int band_min_x = imin_int(t_shape.start_x, t_shape.end_x);
    int band_max_x = imax_int(t_shape.start_x, t_shape.end_x);
    int x;

    if (!intersection_x || !intersection_y) {
        return 0;
    }

    if (den == 0) {
        return 0;
    }

    x = get_line_band_intersection_x(line_x,
                                     control_point.center_y,
                                     line_slope_q8,
                                     t_shape.center_x,
                                     t_shape.center_y,
                                     t_shape.band_slope_q8);

    if (t_shape.left_ext_x >= 0) {
        band_min_x = imin_int(band_min_x, t_shape.left_ext_x);
        band_max_x = imax_int(band_max_x, t_shape.left_ext_x);
    }
    if (t_shape.right_ext_x >= 0) {
        band_min_x = imin_int(band_min_x, t_shape.right_ext_x);
        band_max_x = imax_int(band_max_x, t_shape.right_ext_x);
    }

    if (x < band_min_x - margin || x > band_max_x + margin) {
        return 0;
    }

    if (x < band_min_x) {
        x = band_min_x;
    } else if (x > band_max_x) {
        x = band_max_x;
    }

    *intersection_x = x;
    *intersection_y = get_band_y_at_x(t_shape, x);
    return 1;
}

static int get_line_edge_x_at_y(int edge_x,
                                int edge_y,
                                int edge_slope_q8,
                                int y)
{
    return edge_x + (edge_slope_q8 * (y - edge_y) + 128) / 256;
}

static int point_overruns_x_from_y_line(int point_x,
                                        int point_y,
                                        int line_x,
                                        int line_y,
                                        int line_slope_q8,
                                        int expected_side,
                                        int threshold)
{
    const int line_x_at_point_y = get_line_edge_x_at_y(line_x,
                                                       line_y,
                                                       line_slope_q8,
                                                       point_y);
    const int side_delta = point_x - line_x_at_point_y;
    const long long implicit_line_num =
        (long long)256 * ((long long)point_x - line_x) -
        (long long)line_slope_q8 * ((long long)point_y - line_y);
    const long long distance_num_sq = implicit_line_num * implicit_line_num;
    const long long distance_den = 65536LL + (long long)line_slope_q8 * line_slope_q8;
    const long long threshold_sq = (long long)threshold * threshold;

    if (expected_side < 0 && side_delta >= 0) {
        return 0;
    }

    if (expected_side > 0 && side_delta <= 0) {
        return 0;
    }

    return distance_num_sq > threshold_sq * distance_den;
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

static int find_shape2_row_span(const uint16_t *frame,
                                int width,
                                int height,
                                int row,
                                int *first_x,
                                int *last_x,
                                int *black_count)
{
    const int rotated_width = height;
    int first = -1;
    int last = -1;
    int count = 0;

    for (int col = 0; col < rotated_width; ++col) {
        if (!is_track_pixel(get_px_rotm90_local(frame, width, height, col, row))) {
            continue;
        }

        if (first < 0) {
            first = col;
        }
        last = col;
        count++;
    }

    if (first < 0 || last < first) {
        return 0;
    }

    *first_x = first;
    *last_x = last;
    *black_count = count;
    return 1;
}

static int is_shape2_candidate_row(const uint16_t *frame,
                                   int width,
                                   int height,
                                   line_control_point_t control_point,
                                   int row,
                                   int *first_x,
                                   int *last_x)
{
    const int rotated_width = height;
    const int line_width = imax_int(control_point.width, 6);
    const int min_side_extension = imax_int(SHAPE2_MIN_SIDE_EXTENSION, line_width);
    const int min_total_span = imax_int((rotated_width * 3) / 10, line_width * 3);
    const int expected_center_x = project_line_center_x_at_y(control_point, row);
    const int expected_left_x = expected_center_x - line_width / 2;
    const int expected_right_x = expected_center_x + line_width / 2;
    int first = -1;
    int last = -1;
    int black_count = 0;
    int span;
    int left_extension;
    int right_extension;

    if (!find_shape2_row_span(frame, width, height, row, &first, &last, &black_count)) {
        return 0;
    }

    span = last - first + 1;
    left_extension = expected_left_x - first;
    right_extension = last - expected_right_x;

    if (black_count < imax_int(4, line_width / 2)) {
        return 0;
    }

    if (span >= min_total_span ||
        left_extension >= min_side_extension ||
        right_extension >= min_side_extension) {
        *first_x = first;
        *last_x = last;
        return 1;
    }

    return 0;
}

static shape2_band_run_t find_shape2_band_run(const uint16_t *frame,
                                              int width,
                                              int height,
                                              line_control_point_t control_point)
{
    const int rotated_height = width;
    const int first_search_row = clamp_int_local(control_point.center_y - T_SHAPE_MIN_AHEAD_ROWS,
                                                 0,
                                                 rotated_height - 1);
    shape2_band_run_t best = { 0, -1, -1, height, -1, 0 };
    int run_near_y = -1;
    int run_far_y = -1;
    int run_min_x = height;
    int run_max_x = -1;
    int run_count = 0;
    int gap_count = 0;

    for (int row = first_search_row; row >= 0; --row) {
        int first_x = -1;
        int last_x = -1;

        if (is_shape2_candidate_row(frame, width, height, control_point, row, &first_x, &last_x)) {
            if (run_count == 0) {
                run_near_y = row;
                run_min_x = height;
                run_max_x = -1;
            }

            run_far_y = row;
            if (first_x < run_min_x) run_min_x = first_x;
            if (last_x > run_max_x) run_max_x = last_x;
            run_count++;
            gap_count = 0;
            continue;
        }

        if (run_count > 0 && gap_count < SHAPE2_MAX_BAND_GAP) {
            gap_count++;
            continue;
        }

        if (run_count >= SHAPE2_MIN_BAND_ROWS) {
            best.found = 1;
            best.near_y = run_near_y;
            best.far_y = run_far_y;
            best.min_x = run_min_x;
            best.max_x = run_max_x;
            best.row_count = run_count;
            return best;
        }

        run_near_y = -1;
        run_far_y = -1;
        run_min_x = height;
        run_max_x = -1;
        run_count = 0;
        gap_count = 0;
    }

    if (run_count >= SHAPE2_MIN_BAND_ROWS) {
        best.found = 1;
        best.near_y = run_near_y;
        best.far_y = run_far_y;
        best.min_x = run_min_x;
        best.max_x = run_max_x;
        best.row_count = run_count;
    }

    return best;
}

static t_shape_detection_t fit_shape2_band(const uint16_t *frame,
                                           int width,
                                           int height,
                                           line_control_point_t control_point,
                                           shape2_band_run_t run)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int row_min = clamp_int_local(run.far_y - SHAPE2_BAND_FIT_MARGIN, 0, rotated_height - 1);
    const int row_max = clamp_int_local(run.near_y + SHAPE2_BAND_FIT_MARGIN, 0, rotated_height - 1);
    long sum_top_x = 0;
    long sum_top_y = 0;
    long sum_top_xx = 0;
    long sum_top_xy = 0;
    long sum_bottom_x = 0;
    long sum_bottom_y = 0;
    long sum_bottom_xx = 0;
    long sum_bottom_xy = 0;
    int point_count = 0;
    int min_x = rotated_width;
    int max_x = -1;
    int sum_center_x = 0;
    t_shape_detection_t t_shape = { false, -1, -1, 0, 0, -1, -1, control_point.slope_q8, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0 };

    for (int col = 0; col < rotated_width; ++col) {
        int top_y = -1;
        int bottom_y = -1;

        for (int row = row_min; row <= row_max; ++row) {
            if (!is_track_pixel(get_px_rotm90_local(frame, width, height, col, row))) {
                continue;
            }

            if (top_y < 0) {
                top_y = row;
            }
            bottom_y = row;
        }

        if (top_y < 0 || bottom_y < top_y) {
            continue;
        }

        sum_top_x += col;
        sum_top_y += top_y;
        sum_top_xx += (long)col * col;
        sum_top_xy += (long)col * top_y;
        sum_bottom_x += col;
        sum_bottom_y += bottom_y;
        sum_bottom_xx += (long)col * col;
        sum_bottom_xy += (long)col * bottom_y;
        sum_center_x += col;
        point_count++;
        if (col < min_x) min_x = col;
        if (col > max_x) max_x = col;
    }

    if (point_count < SHAPE2_MIN_BAND_COLUMNS || min_x >= max_x) {
        return t_shape;
    }

    t_shape.found = true;
    t_shape.start_x = min_x;
    t_shape.end_x = max_x;
    t_shape.center_x = (sum_center_x + point_count / 2) / point_count;
    t_shape.center_y = (run.near_y + run.far_y) / 2;
    t_shape.width = max_x - min_x + 1;
    t_shape.thickness = imax_int(1, run.near_y - run.far_y + 1);
    t_shape.band_top_slope_q8 = fit_y_from_x_q8_local(sum_top_x,
                                                      sum_top_y,
                                                      sum_top_xx,
                                                      sum_top_xy,
                                                      point_count);
    t_shape.band_bottom_slope_q8 = fit_y_from_x_q8_local(sum_bottom_x,
                                                         sum_bottom_y,
                                                         sum_bottom_xx,
                                                         sum_bottom_xy,
                                                         point_count);
    t_shape.band_slope_q8 = (t_shape.band_top_slope_q8 + t_shape.band_bottom_slope_q8) / 2;
    t_shape.band_top_x = (int)((sum_top_x + point_count / 2) / point_count);
    t_shape.band_top_y = (int)((sum_top_y + point_count / 2) / point_count);
    t_shape.band_bottom_x = (int)((sum_bottom_x + point_count / 2) / point_count);
    t_shape.band_bottom_y = (int)((sum_bottom_y + point_count / 2) / point_count);
    t_shape.left_ext_x = min_x;
    t_shape.left_ext_y = get_band_y_at_x(t_shape, min_x);
    t_shape.right_ext_x = max_x;
    t_shape.right_ext_y = get_band_y_at_x(t_shape, max_x);
    t_shape.start_proj_q8 = min_x * 256;
    t_shape.end_proj_q8 = max_x * 256;

    return t_shape;
}

static uint16_t *find_line_pos_and_detect_t_shape_2(uint16_t *frame, int width, int height)
{
    line_control_point_t control_point;
    control_point_confidence_t confidence;
    shape2_band_run_t band_run;
    t_shape_detection_t t_shape;
    int shape_candidate = 0;
    int left_overrun = 0;
    int right_overrun = 0;
    int left_intersection_valid = 0;
    int right_intersection_valid = 0;
    int marker_x = -1;
    int marker_y = -1;
    t_shape_detection_t marker_shape;

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

    control_point = sort_line(frame, width, height);
    if (control_point.found) {
        band_run = find_shape2_band_run(frame, width, height, control_point);
        if (band_run.found) {
            control_point = refine_line_slope_between(frame,
                                                      width,
                                                      height,
                                                      control_point,
                                                      band_run.near_y + 1);
            t_shape = fit_shape2_band(frame, width, height, control_point, band_run);
            shape_candidate = t_shape.found;
        } else {
            t_shape = (t_shape_detection_t){ false, -1, -1, 0, 0, -1, -1, control_point.slope_q8, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0 };
        }
    } else {
        band_run = (shape2_band_run_t){ 0, -1, -1, 0, -1, 0 };
        t_shape = (t_shape_detection_t){ false, -1, -1, 0, 0, -1, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0 };
    }

    confidence = take_decision_2(control_point, width, height);
    draw_decided_control_point(frame, width, height, control_point, confidence);

    if (shape_candidate) {
        int left_x = -1;
        int left_y = -1;
        int right_x = -1;
        int right_y = -1;
        const int threshold = imax_int(get_t_shape_edge_extension_min(control_point),
                                       imax_int(control_point.width / 2, 5));

        left_intersection_valid =
            get_line_band_segment_intersection(control_point,
                                               1,
                                               t_shape,
                                               &left_x,
                                               &left_y);
        right_intersection_valid =
            get_line_band_segment_intersection(control_point,
                                               0,
                                               t_shape,
                                               &right_x,
                                               &right_y);

        if (left_intersection_valid) {
            g_left_orange_point_x = left_x;
            g_left_orange_point_y = left_y;
        }
        if (right_intersection_valid) {
            g_right_orange_point_x = right_x;
            g_right_orange_point_y = right_y;
        }
        g_orange_points_valid = left_intersection_valid || right_intersection_valid;

        left_overrun = point_overruns_x_from_y_line(t_shape.left_ext_x,
                                                    t_shape.left_ext_y,
                                                    control_point.start_x,
                                                    control_point.center_y,
                                                    control_point.start_slope_q8,
                                                    LEFT_SIDE,
                                                    threshold);
        right_overrun = point_overruns_x_from_y_line(t_shape.right_ext_x,
                                                     t_shape.right_ext_y,
                                                     control_point.end_x,
                                                     control_point.center_y,
                                                     control_point.end_slope_q8,
                                                     RIGHT_SIDE,
                                                     threshold);

        marker_x = project_line_center_x_at_y(control_point, t_shape.center_y);
        marker_y = t_shape.center_y;
        marker_shape = t_shape;
        marker_shape.center_x = marker_x;
        marker_shape.center_y = marker_y;

        if (left_overrun && right_overrun) {
            publish_t_detection(marker_shape);
        } else if (left_overrun != right_overrun) {
            publish_elbow_detection(left_overrun ? LEFT_SIDE : RIGHT_SIDE, marker_x, marker_y);
        }

        draw_detection_geometry(frame, width, height, control_point, t_shape);
        draw_intersection_cross(frame, width, height, t_shape.left_ext_x, t_shape.left_ext_y, COLOR_RED);
        draw_intersection_cross(frame, width, height, t_shape.right_ext_x, t_shape.right_ext_y, COLOR_RED);
        if (left_intersection_valid) {
            draw_intersection_cross(frame, width, height, g_left_orange_point_x, g_left_orange_point_y, COLOR_ORANGE);
        }
        if (right_intersection_valid) {
            draw_intersection_cross(frame, width, height, g_right_orange_point_x, g_right_orange_point_y, COLOR_ORANGE);
        }

        if (left_overrun && right_overrun) {
            draw_t_shape_marker(frame, width, height, marker_shape, COLOR_RED);
        } else if (left_overrun != right_overrun) {
            draw_elbow_marker(frame,
                              width,
                              height,
                              marker_x,
                              marker_y,
                              left_overrun ? LEFT_SIDE : RIGHT_SIDE,
                              control_point.slope_q8,
                              COLOR_RED);
        }
    }

    return frame;
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
    int left_intersection_valid = 0;
    int right_intersection_valid = 0;
    const int marker_x = project_line_center_x_at_y(control_point, t_shape.center_y);
    const int marker_y = t_shape.center_y;
    t_shape_detection_t marker_shape = t_shape;

    if (shape_candidate) {
        int left_x = -1;
        int left_y = -1;
        int right_x = -1;
        int right_y = -1;
        left_intersection_valid =
            get_line_band_segment_intersection(control_point,
                                               1,
                                               t_shape,
                                               &left_x,
                                               &left_y);
        right_intersection_valid =
            get_line_band_segment_intersection(control_point,
                                               0,
                                               t_shape,
                                               &right_x,
                                               &right_y);
        g_orange_points_valid = left_intersection_valid || right_intersection_valid;
        if (left_intersection_valid) {
            g_left_orange_point_x = left_x;
            g_left_orange_point_y = left_y;
        }
        if (right_intersection_valid) {
            g_right_orange_point_x = right_x;
            g_right_orange_point_y = right_y;
        }
    }

    if (shape_candidate) {
        const int threshold = get_t_shape_edge_extension_min(control_point);

        left_overrun = point_overruns_x_from_y_line(t_shape.left_ext_x,
                                                    t_shape.left_ext_y,
                                                    control_point.start_x,
                                                    control_point.center_y,
                                                    control_point.start_slope_q8,
                                                    LEFT_SIDE,
                                                    threshold);
        right_overrun = point_overruns_x_from_y_line(t_shape.right_ext_x,
                                                     t_shape.right_ext_y,
                                                     control_point.end_x,
                                                     control_point.center_y,
                                                     control_point.end_slope_q8,
                                                     RIGHT_SIDE,
                                                     threshold);
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

            if (left_intersection_valid) {
                draw_intersection_cross(frame,
                                        width,
                                        height,
                                        g_left_orange_point_x,
                                        g_left_orange_point_y,
                                        COLOR_ORANGE);
            }
            if (right_intersection_valid) {
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
            if (left_intersection_valid) {
                draw_intersection_cross(frame,
                                        width,
                                        height,
                                        left_mid_x,
                                        left_mid_y,
                                        COLOR_GREEN);
            }
            if (right_intersection_valid) {
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
    return find_line_pos_and_detect_t_shape_2(frame, width, height);
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
