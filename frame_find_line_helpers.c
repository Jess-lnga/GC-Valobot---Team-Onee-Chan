// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#include "frame_find_line_helpers.h"

#define BRIGHTNESS_HARD_BLACK_MAX     26
#define BRIGHTNESS_DYNAMIC_MARGIN      8
#define CHROMA_BLACK_MAX              10
#define CHROMA_NEAR_BLACK_MAX         16
#define SATURATION_BLACK_LIMIT        14
#define MIN_DYNAMIC_BLACK_THRESHOLD   24
#define MAX_DYNAMIC_BLACK_THRESHOLD   60

#define MIN_BLACK_SEGMENT_WIDTH        6
#define MAX_WHITE_GAP_IN_SEGMENT       3
#define CONTROL_BAND_HEIGHT           24
#define MAX_SEGMENTS_PER_ROW           8
#define MAX_ROW_CENTER_JUMP          24
#define LINE_SEGMENT_MAX_WIDTH_RATIO_NUM 1
#define LINE_SEGMENT_MAX_WIDTH_RATIO_DEN 2
#define T_SHAPE_MIN_WIDTH_RATIO_NUM    1
#define T_SHAPE_MIN_WIDTH_RATIO_DEN    2
#define T_SHAPE_MIN_ABSOLUTE_WIDTH    40
#define T_SHAPE_MIN_ROW_BLACK_RATIO_NUM 2
#define T_SHAPE_MIN_ROW_BLACK_RATIO_DEN 3
#define T_SHAPE_MIN_THICKNESS          4
#define T_SHAPE_BORDER_MARGIN          4
#define T_SHAPE_MIN_VERTICAL_FIT_POINTS 4
#define LINE_REFINE_MAX_CENTER_DELTA  30

typedef struct {
    int start_x;
    int end_x;
    int center_x;
    int width;
} black_segment_t;

static inline int rotm90_xo(int xv, int yv)
{
    (void)xv;
    return yv;
}

static inline int rotm90_yo(int xv, int height)
{
    return (height - 1) - xv;
}

static inline uint16_t get_px_rotm90(const uint16_t *frame, int width, int height, int xv, int yv)
{
    const int xo = rotm90_xo(xv, yv);
    const int yo = rotm90_yo(xv, height);
    return frame[yo * width + xo];
}

static inline void set_px_rotm90(uint16_t *frame, int width, int height, int xv, int yv, uint16_t color)
{
    const int xo = rotm90_xo(xv, yv);
    const int yo = rotm90_yo(xv, height);
    frame[yo * width + xo] = color;
}

static inline void rgb565_to_components(uint16_t px,
                                        uint8_t *r5,
                                        uint8_t *g6,
                                        uint8_t *b5)
{
    *r5 = (px >> 11) & 0x1F;
    *g6 = (px >> 5) & 0x3F;
    *b5 = px & 0x1F;
}

static inline int compute_brightness(uint8_t r5, uint8_t g6, uint8_t b5)
{
    return (int)r5 + (int)g6 + (int)b5;
}

static inline int compute_chroma(uint8_t r5, uint8_t g6, uint8_t b5)
{
    int maxv = r5;
    int minv = r5;

    if (g6 > maxv) maxv = g6;
    if (b5 > maxv) maxv = b5;

    if (g6 < minv) minv = g6;
    if (b5 < minv) minv = b5;

    return maxv - minv;
}

static int clamp_int(int x, int xmin, int xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static int project_x_q8(int x, int y, int slope_q8)
{
    return x * 256 - slope_q8 * y;
}

static int unproject_x_at_y(int proj_q8, int y, int slope_q8)
{
    return (proj_q8 + slope_q8 * y + 128) / 256;
}

static int y_on_x_line_q8(int x0, int y0, int slope_q8, int x)
{
    return y0 + (slope_q8 * (x - x0) + 128) / 256;
}

static int fit_x_from_y_q8(long sum_x, long sum_y, long sum_yy, long sum_xy, int count)
{
    const long den = (long)count * sum_yy - sum_y * sum_y;

    if (count < 2 || den == 0) {
        return 0;
    }

    {
        const long num = (long)count * sum_xy - sum_y * sum_x;
        return clamp_int((int)((num * 256 + den / 2) / den), -1024, 1024);
    }
}

static int fit_y_from_x_q8(long sum_x, long sum_y, long sum_xx, long sum_xy, int count)
{
    const long den = (long)count * sum_xx - sum_x * sum_x;

    if (count < 2 || den == 0) {
        return 0;
    }

    {
        const long num = (long)count * sum_xy - sum_x * sum_y;
        return clamp_int((int)((num * 256 + den / 2) / den), -1024, 1024);
    }
}

static bool is_near_image_border(int x, int y, int rotated_width, int rotated_height)
{
    return x < T_SHAPE_BORDER_MARGIN ||
           x >= rotated_width - T_SHAPE_BORDER_MARGIN ||
           y < T_SHAPE_BORDER_MARGIN ||
           y >= rotated_height - T_SHAPE_BORDER_MARGIN;
}

static void fit_t_shape_band_edges_from_vertical_segments(const uint16_t *frame,
                                                          int width,
                                                          int height,
                                                          int col_min,
                                                          int col_max,
                                                          int row_min,
                                                          int row_max,
                                                          t_shape_detection_t *t_shape)
{
    const int rotated_width = height;
    const int rotated_height = width;
    long sum_top_x = 0;
    long sum_top_y = 0;
    long sum_top_xx = 0;
    long sum_top_xy = 0;
    long sum_bottom_x = 0;
    long sum_bottom_y = 0;
    long sum_bottom_xx = 0;
    long sum_bottom_xy = 0;
    int top_count = 0;
    int bottom_count = 0;

    col_min = clamp_int(col_min, 0, rotated_width - 1);
    col_max = clamp_int(col_max, 0, rotated_width - 1);
    row_min = clamp_int(row_min, 0, rotated_height - 1);
    row_max = clamp_int(row_max, 0, rotated_height - 1);

    if (col_max <= col_min || row_max <= row_min || !t_shape) {
        return;
    }

    for (int col = col_min; col <= col_max; ++col) {
        int first_black_y = -1;
        int last_black_y = -1;

        if (col < T_SHAPE_BORDER_MARGIN ||
            col >= rotated_width - T_SHAPE_BORDER_MARGIN) {
            continue;
        }

        for (int row = row_min; row <= row_max; ++row) {
            if (get_px_rotm90(frame, width, height, col, row) != COLOR_BLACK) {
                continue;
            }

            if (first_black_y < 0) {
                first_black_y = row;
            }
            last_black_y = row;
        }

        if (first_black_y < 0 || last_black_y < first_black_y) {
            continue;
        }

        if (!is_near_image_border(col, first_black_y, rotated_width, rotated_height)) {
            sum_top_x += col;
            sum_top_y += first_black_y;
            sum_top_xx += (long)col * col;
            sum_top_xy += (long)col * first_black_y;
            top_count++;
        }

        if (!is_near_image_border(col, last_black_y, rotated_width, rotated_height)) {
            sum_bottom_x += col;
            sum_bottom_y += last_black_y;
            sum_bottom_xx += (long)col * col;
            sum_bottom_xy += (long)col * last_black_y;
            bottom_count++;
        }
    }

    if (top_count >= T_SHAPE_MIN_VERTICAL_FIT_POINTS &&
        bottom_count >= T_SHAPE_MIN_VERTICAL_FIT_POINTS) {
        t_shape->band_top_slope_q8 = fit_y_from_x_q8(sum_top_x,
                                                     sum_top_y,
                                                     sum_top_xx,
                                                     sum_top_xy,
                                                     top_count);
        t_shape->band_bottom_slope_q8 = fit_y_from_x_q8(sum_bottom_x,
                                                        sum_bottom_y,
                                                        sum_bottom_xx,
                                                        sum_bottom_xy,
                                                        bottom_count);
        t_shape->band_slope_q8 = (t_shape->band_top_slope_q8 + t_shape->band_bottom_slope_q8) / 2;
        t_shape->band_top_x = (int)((sum_top_x + top_count / 2) / top_count);
        t_shape->band_top_y = (int)((sum_top_y + top_count / 2) / top_count);
        t_shape->band_bottom_x = (int)((sum_bottom_x + bottom_count / 2) / bottom_count);
        t_shape->band_bottom_y = (int)((sum_bottom_y + bottom_count / 2) / bottom_count);
    }
}

static int compute_dynamic_black_threshold(const uint16_t *frame, int width, int height)
{
    int min_brightness = 255;
    int dark_sum = 0;
    int dark_count = 0;
    const int sample_step = 4;

    for (int y = 0; y < height; y += sample_step) {
        int row_offset = y * width;

        for (int x = 0; x < width; x += sample_step) {
            uint8_t r5, g6, b5;
            rgb565_to_components(frame[row_offset + x], &r5, &g6, &b5);

            int brightness = compute_brightness(r5, g6, b5);
            if (brightness < min_brightness) {
                min_brightness = brightness;
            }

            if (brightness <= 72) {
                dark_sum += brightness;
                dark_count++;
            }
        }
    }

    if (dark_count > 0) {
        int avg_dark = dark_sum / dark_count;
        return clamp_int(avg_dark + BRIGHTNESS_DYNAMIC_MARGIN,
                         MIN_DYNAMIC_BLACK_THRESHOLD,
                         MAX_DYNAMIC_BLACK_THRESHOLD);
    }

    return clamp_int(min_brightness + BRIGHTNESS_DYNAMIC_MARGIN,
                     MIN_DYNAMIC_BLACK_THRESHOLD,
                     MAX_DYNAMIC_BLACK_THRESHOLD);
}

static bool is_black_pixel(uint16_t px, int dynamic_threshold)
{
    uint8_t r5, g6, b5;
    rgb565_to_components(px, &r5, &g6, &b5);

    const int brightness = compute_brightness(r5, g6, b5);
    const int chroma = compute_chroma(r5, g6, b5);

    if (brightness <= BRIGHTNESS_HARD_BLACK_MAX && chroma <= CHROMA_NEAR_BLACK_MAX) {
        return true;
    }

    if (brightness > dynamic_threshold) {
        return false;
    }

    if (chroma > CHROMA_BLACK_MAX) {
        return false;
    }

    if (((int)g6 - (int)r5 > SATURATION_BLACK_LIMIT) ||
        ((int)g6 - (int)b5 > SATURATION_BLACK_LIMIT)) {
        return false;
    }

    return true;
}

void filter_black_pxl(uint16_t *frame, int width, int height)
{
    const int black_threshold = compute_dynamic_black_threshold(frame, width, height);
    const int pixel_count = width * height;

    for (int i = 0; i < pixel_count; ++i) {
        frame[i] = is_black_pixel(frame[i], black_threshold) ? COLOR_BLACK : COLOR_WHITE;
    }
}

static void mark_segment(uint16_t *frame,
                         int width,
                         int height,
                         int row,
                         int start_x,
                         int end_x)
{
    const int center_x = (start_x + end_x) / 2;

    set_px_rotm90(frame, width, height, start_x, row, COLOR_BLUE);
    set_px_rotm90(frame, width, height, end_x, row, COLOR_BLUE);
    set_px_rotm90(frame, width, height, center_x, row, COLOR_GREEN);
}

static int iabs_int(int x)
{
    return (x < 0) ? -x : x;
}

static int get_max_line_segment_width(int rotated_width)
{
    return (rotated_width * LINE_SEGMENT_MAX_WIDTH_RATIO_NUM) /
           LINE_SEGMENT_MAX_WIDTH_RATIO_DEN;
}

static bool is_line_segment_width_valid(int segment_width, int rotated_width)
{
    return segment_width <= get_max_line_segment_width(rotated_width);
}

static void register_segment_candidate(black_segment_t *segments,
                                       int *segment_count,
                                       int start_x,
                                       int end_x)
{
    if (*segment_count >= MAX_SEGMENTS_PER_ROW) {
        return;
    }

    segments[*segment_count].start_x = start_x;
    segments[*segment_count].end_x = end_x;
    segments[*segment_count].center_x = (start_x + end_x) / 2;
    segments[*segment_count].width = end_x - start_x + 1;
    (*segment_count)++;
}

static int choose_segment_index(const black_segment_t *segments,
                                int segment_count,
                                int reference_center_x,
                                int has_reference)
{
    int best_idx = -1;
    int best_score = 0x7FFFFFFF;

    for (int i = 0; i < segment_count; ++i) {
        const int dx = iabs_int(segments[i].center_x - reference_center_x);

        if (has_reference && dx > MAX_ROW_CENTER_JUMP) {
            continue;
        }

        if (dx < best_score) {
            best_score = dx;
            best_idx = i;
        }
    }

    if (best_idx >= 0) {
        return best_idx;
    }

    for (int i = 0; i < segment_count; ++i) {
        const int dx = iabs_int(segments[i].center_x - reference_center_x);
        if (dx < best_score) {
            best_score = dx;
            best_idx = i;
        }
    }

    return best_idx;
}

void find_black_segments(uint16_t *frame, int width, int height)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int image_center_x = rotated_width / 2;
    int previous_center_x = image_center_x;
    int has_previous_center = 0;

    for (int row = 0; row < rotated_height; ++row) {
        int segment_start = -1;
        int last_black_x = -1;
        int white_gap = 0;
        black_segment_t segments[MAX_SEGMENTS_PER_ROW];
        int segment_count = 0;

        for (int col = 0; col < rotated_width; ++col) {
            const bool is_black = (get_px_rotm90(frame, width, height, col, row) == COLOR_BLACK);

            if (is_black) {
                if (segment_start < 0) {
                    segment_start = col;
                }

                last_black_x = col;
                white_gap = 0;
                continue;
            }

            if (segment_start < 0) {
                continue;
            }

            white_gap++;
            if (white_gap <= MAX_WHITE_GAP_IN_SEGMENT) {
                continue;
            }

            if (last_black_x >= segment_start) {
                const int segment_width = last_black_x - segment_start + 1;
                if (segment_width >= MIN_BLACK_SEGMENT_WIDTH &&
                    is_line_segment_width_valid(segment_width, rotated_width)) {
                    register_segment_candidate(segments, &segment_count, segment_start, last_black_x);
                }
            }

            segment_start = -1;
            last_black_x = -1;
            white_gap = 0;
        }

        if (segment_start >= 0 && last_black_x >= segment_start) {
            const int segment_width = last_black_x - segment_start + 1;
            if (segment_width >= MIN_BLACK_SEGMENT_WIDTH &&
                is_line_segment_width_valid(segment_width, rotated_width)) {
                register_segment_candidate(segments, &segment_count, segment_start, last_black_x);
            }
        }

        if (segment_count > 0) {
            const int chosen_idx = choose_segment_index(segments,
                                                        segment_count,
                                                        has_previous_center ? previous_center_x : image_center_x,
                                                        has_previous_center);
            if (chosen_idx >= 0) {
                mark_segment(frame,
                             width,
                             height,
                             row,
                             segments[chosen_idx].start_x,
                             segments[chosen_idx].end_x);
                previous_center_x = segments[chosen_idx].center_x;
                has_previous_center = 1;
            }
        }
    }
}

void draw_control_point(uint16_t *frame, int width, int height, int center_x, int center_y, uint16_t color)
{
    set_px_rotm90(frame, width, height, center_x, center_y, color);

    if (center_x > 0) {
        set_px_rotm90(frame, width, height, center_x - 1, center_y, color);
    }
    if (center_x + 1 < height) {
        set_px_rotm90(frame, width, height, center_x + 1, center_y, color);
    }
    if (center_y > 0) {
        set_px_rotm90(frame, width, height, center_x, center_y - 1, color);
    }
    if (center_y + 1 < width) {
        set_px_rotm90(frame, width, height, center_x, center_y + 1, color);
    }
}

static t_shape_detection_t make_t_shape_detection(const uint16_t *frame,
                                                  int width,
                                                  int height,
                                                  int run_start_row,
                                                  int run_end_row,
                                                  int run_min_x,
                                                  int run_max_x,
                                                  int run_min_proj_q8,
                                                  int run_max_proj_q8,
                                                  int run_sum_center_x,
                                                  long run_sum_x,
                                                  long run_sum_y,
                                                  long run_sum_xx,
                                                  long run_sum_xy,
                                                  long run_sum_left_x,
                                                  long run_sum_left_y,
                                                  long run_sum_left_xx,
                                                  long run_sum_left_xy,
                                                  int run_left_count,
                                                  long run_sum_right_x,
                                                  long run_sum_right_y,
                                                  long run_sum_right_xx,
                                                  long run_sum_right_xy,
                                                  int run_right_count,
                                                  int run_row_count,
                                                  int line_slope_q8)
{
    t_shape_detection_t output = { false, -1, -1, 0, 0, -1, -1, line_slope_q8, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0 };
    int center_y;
    const int left_slope_q8 = fit_y_from_x_q8(run_sum_left_x,
                                              run_sum_left_y,
                                              run_sum_left_xx,
                                              run_sum_left_xy,
                                              run_left_count);
    const int right_slope_q8 = fit_y_from_x_q8(run_sum_right_x,
                                               run_sum_right_y,
                                               run_sum_right_xx,
                                               run_sum_right_xy,
                                               run_right_count);

    if (run_row_count < T_SHAPE_MIN_THICKNESS) {
        return output;
    }

    center_y = (run_start_row + run_end_row) / 2;

    output.found = true;
    output.center_x = (run_sum_center_x + run_row_count / 2) / run_row_count;
    output.center_y = center_y;
    output.width = (run_max_proj_q8 - run_min_proj_q8 + 128) / 256;
    output.thickness = run_row_count;
    output.start_x = unproject_x_at_y(run_min_proj_q8, center_y, line_slope_q8);
    output.end_x = unproject_x_at_y(run_max_proj_q8, center_y, line_slope_q8);
    output.slope_q8 = line_slope_q8;
    output.band_top_slope_q8 = left_slope_q8;
    output.band_bottom_slope_q8 = right_slope_q8;
    output.band_slope_q8 = (left_slope_q8 + right_slope_q8) / 2;
    output.band_top_x = output.center_x;
    output.band_top_y = output.center_y;
    output.band_bottom_x = output.center_x;
    output.band_bottom_y = output.center_y;
    if (run_left_count > 0) {
        output.left_ext_x = (int)((run_sum_left_x + run_left_count / 2) / run_left_count);
        output.left_ext_y = (int)((run_sum_left_y + run_left_count / 2) / run_left_count);
    } else {
        output.left_ext_x = output.start_x;
        output.left_ext_y = output.center_y;
    }
    if (run_right_count > 0) {
        output.right_ext_x = (int)((run_sum_right_x + run_right_count / 2) / run_right_count);
        output.right_ext_y = (int)((run_sum_right_y + run_right_count / 2) / run_right_count);
    } else {
        output.right_ext_x = output.end_x;
        output.right_ext_y = output.center_y;
    }
    fit_t_shape_band_edges_from_vertical_segments(frame,
                                                  width,
                                                  height,
                                                  run_min_x,
                                                  run_max_x,
                                                  (run_start_row < run_end_row) ? run_start_row : run_end_row,
                                                  (run_start_row > run_end_row) ? run_start_row : run_end_row,
                                                  &output);
    if (output.band_top_x >= 0 &&
        output.band_top_y >= 0 &&
        output.band_bottom_x >= 0 &&
        output.band_bottom_y >= 0) {
        const int left_top_y = y_on_x_line_q8(output.band_top_x,
                                              output.band_top_y,
                                              output.band_top_slope_q8,
                                              output.start_x);
        const int left_bottom_y = y_on_x_line_q8(output.band_bottom_x,
                                                 output.band_bottom_y,
                                                 output.band_bottom_slope_q8,
                                                 output.start_x);
        const int right_top_y = y_on_x_line_q8(output.band_top_x,
                                               output.band_top_y,
                                               output.band_top_slope_q8,
                                               output.end_x);
        const int right_bottom_y = y_on_x_line_q8(output.band_bottom_x,
                                                  output.band_bottom_y,
                                                  output.band_bottom_slope_q8,
                                                  output.end_x);

        output.left_ext_x = output.start_x;
        output.left_ext_y = (left_top_y + left_bottom_y) / 2;
        output.right_ext_x = output.end_x;
        output.right_ext_y = (right_top_y + right_bottom_y) / 2;
    }
    output.start_proj_q8 = run_min_proj_q8;
    output.end_proj_q8 = run_max_proj_q8;

    if (output.start_x > output.end_x) {
        const int tmp = output.start_x;
        output.start_x = output.end_x;
        output.end_x = tmp;
    }

    if (output.width <= 0) {
        output.width = run_max_x - run_min_x + 1;
    }

    return output;
}

t_shape_detection_t detect_t_shape(uint16_t *frame,
                                   int width,
                                   int height,
                                   int line_slope_q8,
                                   int nearest_search_row)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int first_row = clamp_int(nearest_search_row, 0, rotated_height - 1);
    const int min_wide_span = clamp_int((rotated_width * T_SHAPE_MIN_WIDTH_RATIO_NUM)
                                      / T_SHAPE_MIN_WIDTH_RATIO_DEN,
                                      T_SHAPE_MIN_ABSOLUTE_WIDTH,
                                      rotated_width);
    t_shape_detection_t best = { false, -1, -1, 0, 0, -1, -1, line_slope_q8, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0 };

    int run_start_row = -1;
    int run_end_row = -1;
    int run_min_x = rotated_width;
    int run_max_x = -1;
    int run_min_proj_q8 = 0x7FFFFFFF;
    int run_max_proj_q8 = -0x7FFFFFFF;
    int run_sum_center_x = 0;
    long run_sum_x = 0;
    long run_sum_y = 0;
    long run_sum_xx = 0;
    long run_sum_xy = 0;
    long run_sum_left_x = 0;
    long run_sum_left_y = 0;
    long run_sum_left_xx = 0;
    long run_sum_left_xy = 0;
    int run_left_count = 0;
    long run_sum_right_x = 0;
    long run_sum_right_y = 0;
    long run_sum_right_xx = 0;
    long run_sum_right_xy = 0;
    int run_right_count = 0;
    int run_row_count = 0;

    for (int row = first_row; row >= 0; --row) {
        int first_black_x = -1;
        int last_black_x = -1;
        int black_count = 0;

        for (int col = 0; col < rotated_width; ++col) {
            if (get_px_rotm90(frame, width, height, col, row) != COLOR_BLACK) {
                continue;
            }

            if (first_black_x < 0) {
                first_black_x = col;
            }
            last_black_x = col;
            black_count++;
        }

        const int span_width = (first_black_x >= 0) ? (last_black_x - first_black_x + 1) : 0;
        const int first_proj_q8 = (first_black_x >= 0) ?
                                  project_x_q8(first_black_x, row, line_slope_q8) : 0;
        const int last_proj_q8 = (first_black_x >= 0) ?
                                 project_x_q8(last_black_x, row, line_slope_q8) : 0;
        const int proj_span_width = (first_black_x >= 0) ?
                                    ((last_proj_q8 - first_proj_q8 + 128) / 256 + 1) : 0;
        const bool is_wide_black_row =
            (proj_span_width >= min_wide_span) &&
            (black_count * T_SHAPE_MIN_ROW_BLACK_RATIO_DEN >=
             span_width * T_SHAPE_MIN_ROW_BLACK_RATIO_NUM);

        if (is_wide_black_row) {
            const int center_x = (first_black_x + last_black_x) / 2;

            if (run_start_row < 0) {
                run_start_row = row;
                run_min_x = first_black_x;
                run_max_x = last_black_x;
                run_min_proj_q8 = 0x7FFFFFFF;
                run_max_proj_q8 = -0x7FFFFFFF;
                run_sum_center_x = 0;
                run_sum_x = 0;
                run_sum_y = 0;
                run_sum_xx = 0;
                run_sum_xy = 0;
                run_sum_left_x = 0;
                run_sum_left_y = 0;
                run_sum_left_xx = 0;
                run_sum_left_xy = 0;
                run_left_count = 0;
                run_sum_right_x = 0;
                run_sum_right_y = 0;
                run_sum_right_xx = 0;
                run_sum_right_xy = 0;
                run_right_count = 0;
                run_row_count = 0;
            }

            run_end_row = row;
            if (first_black_x < run_min_x) run_min_x = first_black_x;
            if (last_black_x > run_max_x) run_max_x = last_black_x;
            if (first_proj_q8 < run_min_proj_q8) run_min_proj_q8 = first_proj_q8;
            if (first_proj_q8 > run_max_proj_q8) run_max_proj_q8 = first_proj_q8;
            if (last_proj_q8 < run_min_proj_q8) run_min_proj_q8 = last_proj_q8;
            if (last_proj_q8 > run_max_proj_q8) run_max_proj_q8 = last_proj_q8;
            run_sum_center_x += center_x;
            run_sum_x += center_x;
            run_sum_y += row;
            run_sum_xx += (long)center_x * center_x;
            run_sum_xy += (long)center_x * row;
            if (!is_near_image_border(first_black_x, row, rotated_width, rotated_height)) {
                run_sum_left_x += first_black_x;
                run_sum_left_y += row;
                run_sum_left_xx += (long)first_black_x * first_black_x;
                run_sum_left_xy += (long)first_black_x * row;
                run_left_count++;
            }
            if (!is_near_image_border(last_black_x, row, rotated_width, rotated_height)) {
                run_sum_right_x += last_black_x;
                run_sum_right_y += row;
                run_sum_right_xx += (long)last_black_x * last_black_x;
                run_sum_right_xy += (long)last_black_x * row;
                run_right_count++;
            }
            run_row_count++;
            continue;
        }

        if (run_row_count >= T_SHAPE_MIN_THICKNESS) {
            return make_t_shape_detection(frame,
                                          width,
                                          height,
                                          run_start_row,
                                          run_end_row,
                                          run_min_x,
                                          run_max_x,
                                          run_min_proj_q8,
                                          run_max_proj_q8,
                                          run_sum_center_x,
                                          run_sum_x,
                                          run_sum_y,
                                          run_sum_xx,
                                          run_sum_xy,
                                          run_sum_left_x,
                                          run_sum_left_y,
                                          run_sum_left_xx,
                                          run_sum_left_xy,
                                          run_left_count,
                                          run_sum_right_x,
                                          run_sum_right_y,
                                          run_sum_right_xx,
                                          run_sum_right_xy,
                                          run_right_count,
                                          run_row_count,
                                          line_slope_q8);
        }

        run_start_row = -1;
        run_end_row = -1;
        run_min_x = rotated_width;
        run_max_x = -1;
        run_min_proj_q8 = 0x7FFFFFFF;
        run_max_proj_q8 = -0x7FFFFFFF;
        run_sum_center_x = 0;
        run_sum_x = 0;
        run_sum_y = 0;
        run_sum_xx = 0;
        run_sum_xy = 0;
        run_sum_left_x = 0;
        run_sum_left_y = 0;
        run_sum_left_xx = 0;
        run_sum_left_xy = 0;
        run_left_count = 0;
        run_sum_right_x = 0;
        run_sum_right_y = 0;
        run_sum_right_xx = 0;
        run_sum_right_xy = 0;
        run_right_count = 0;
        run_row_count = 0;
    }

    if (run_row_count >= T_SHAPE_MIN_THICKNESS) {
        return make_t_shape_detection(frame,
                                      width,
                                      height,
                                      run_start_row,
                                      run_end_row,
                                      run_min_x,
                                      run_max_x,
                                      run_min_proj_q8,
                                      run_max_proj_q8,
                                      run_sum_center_x,
                                      run_sum_x,
                                      run_sum_y,
                                      run_sum_xx,
                                      run_sum_xy,
                                      run_sum_left_x,
                                      run_sum_left_y,
                                      run_sum_left_xx,
                                      run_sum_left_xy,
                                      run_left_count,
                                      run_sum_right_x,
                                      run_sum_right_y,
                                      run_sum_right_xx,
                                      run_sum_right_xy,
                                      run_right_count,
                                      run_row_count,
                                      line_slope_q8);
    }

    return best;
}

void draw_t_shape_marker(uint16_t *frame, int width, int height, t_shape_detection_t t_shape, uint16_t color)
{
    if (!t_shape.found) {
        return;
    }

    const int rotated_width = height;
    const int rotated_height = width;
    const int half_bar = 8;
    const int stem_len = 12;
    const int x0 = clamp_int(t_shape.center_x, 0, rotated_width - 1);
    const int y0 = clamp_int(t_shape.center_y, 0, rotated_height - 1);
    const int slope_q8 = t_shape.slope_q8;

    for (int dx = -half_bar; dx <= half_bar; ++dx) {
        const int x = x0 + dx;
        const int y = y0 - (slope_q8 * dx + 128) / 256;
        if (x >= 0 && x < rotated_width && y >= 0 && y < rotated_height) {
            set_px_rotm90(frame, width, height, x, y, color);
            if (y + 1 < rotated_height) {
                set_px_rotm90(frame, width, height, x, y + 1, color);
            }
        }
    }

    for (int dy = 0; dy <= stem_len; ++dy) {
        const int y = y0 + dy;
        const int x = x0 + (slope_q8 * dy + 128) / 256;
        if (x >= 0 && x < rotated_width && y >= 0 && y < rotated_height) {
            set_px_rotm90(frame, width, height, x, y, color);
            if (x + 1 < rotated_width) {
                set_px_rotm90(frame, width, height, x + 1, y, color);
            }
        }
    }
}

void draw_elbow_marker(uint16_t *frame,
                       int width,
                       int height,
                       int center_x,
                       int center_y,
                       int direction,
                       int slope_q8,
                       uint16_t color)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int x0 = clamp_int(center_x, 0, rotated_width - 1);
    const int y0 = clamp_int(center_y, 0, rotated_height - 1);
    const int dir = (direction < 0) ? -1 : 1;
    const int branch_len = 16;
    const int stem_len = 12;

    for (int dy = 0; dy <= stem_len; ++dy) {
        const int y = y0 + dy;
        const int x = x0 + (slope_q8 * dy + 128) / 256;
        if (x >= 0 && x < rotated_width && y >= 0 && y < rotated_height) {
            set_px_rotm90(frame, width, height, x, y, color);
            if (x + 1 < rotated_width) {
                set_px_rotm90(frame, width, height, x + 1, y, color);
            }
        }
    }

    for (int dx = 0; dx <= branch_len; ++dx) {
        const int x = x0 + dir * dx;
        const int y = y0 - (slope_q8 * dir * dx + 128) / 256;
        if (x >= 0 && x < rotated_width && y >= 0 && y < rotated_height) {
            set_px_rotm90(frame, width, height, x, y, color);
            if (y + 1 < rotated_height) {
                set_px_rotm90(frame, width, height, x, y + 1, color);
            }
        }
    }
}

static void draw_x_from_y_line(uint16_t *frame,
                               int width,
                               int height,
                               int x0,
                               int y0,
                               int slope_q8,
                               int y_start,
                               int y_end,
                               uint16_t color)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int ymin = clamp_int((y_start < y_end) ? y_start : y_end, 0, rotated_height - 1);
    const int ymax = clamp_int((y_start > y_end) ? y_start : y_end, 0, rotated_height - 1);

    for (int y = ymin; y <= ymax; ++y) {
        const int x = x0 + (slope_q8 * (y - y0) + 128) / 256;
        if (x >= 0 && x < rotated_width) {
            set_px_rotm90(frame, width, height, x, y, color);
        }
    }
}

static void draw_y_from_x_line(uint16_t *frame,
                               int width,
                               int height,
                               int x0,
                               int y0,
                               int slope_q8,
                               int x_start,
                               int x_end,
                               uint16_t color)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int xmin = clamp_int((x_start < x_end) ? x_start : x_end, 0, rotated_width - 1);
    const int xmax = clamp_int((x_start > x_end) ? x_start : x_end, 0, rotated_width - 1);

    for (int x = xmin; x <= xmax; ++x) {
        const int y = y0 + (slope_q8 * (x - x0) + 128) / 256;
        if (y >= 0 && y < rotated_height) {
            set_px_rotm90(frame, width, height, x, y, color);
        }
    }
}

void draw_detection_geometry(uint16_t *frame,
                             int width,
                             int height,
                             line_control_point_t control_point,
                             t_shape_detection_t t_shape)
{
    if (!control_point.found || !t_shape.found) {
        return;
    }

    draw_x_from_y_line(frame,
                       width,
                       height,
                       control_point.center_x,
                       control_point.center_y,
                       control_point.slope_q8,
                       t_shape.center_y,
                       control_point.center_y,
                       COLOR_GREEN);
    draw_x_from_y_line(frame,
                       width,
                       height,
                       control_point.start_x,
                       control_point.center_y,
                       control_point.start_slope_q8,
                       t_shape.center_y,
                       control_point.center_y,
                       COLOR_BLUE);
    draw_x_from_y_line(frame,
                       width,
                       height,
                       control_point.end_x,
                       control_point.center_y,
                       control_point.end_slope_q8,
                       t_shape.center_y,
                       control_point.center_y,
                       COLOR_BLUE);
    draw_y_from_x_line(frame,
                       width,
                       height,
                       t_shape.center_x,
                       t_shape.center_y,
                       t_shape.band_slope_q8,
                       t_shape.start_x,
                       t_shape.end_x,
                       COLOR_YELLOW);

    if (t_shape.band_top_x >= 0 && t_shape.band_top_y >= 0) {
        draw_y_from_x_line(frame,
                           width,
                           height,
                           t_shape.band_top_x,
                           t_shape.band_top_y,
                           t_shape.band_top_slope_q8,
                           t_shape.start_x,
                           t_shape.end_x,
                           COLOR_PURPLE);
    }

    if (t_shape.band_bottom_x >= 0 && t_shape.band_bottom_y >= 0) {
        draw_y_from_x_line(frame,
                           width,
                           height,
                           t_shape.band_bottom_x,
                           t_shape.band_bottom_y,
                           t_shape.band_bottom_slope_q8,
                           t_shape.start_x,
                           t_shape.end_x,
                           COLOR_PURPLE);
    }
}

void draw_intersection_cross(uint16_t *frame, int width, int height, int center_x, int center_y, uint16_t color)
{
    const int rotated_width = height;
    const int rotated_height = width;

    for (int d = -3; d <= 3; ++d) {
        const int x1 = center_x + d;
        const int y1 = center_y + d;
        const int x2 = center_x + d;
        const int y2 = center_y - d;

        if (x1 >= 0 && x1 < rotated_width && y1 >= 0 && y1 < rotated_height) {
            set_px_rotm90(frame, width, height, x1, y1, color);
        }
        if (x2 >= 0 && x2 < rotated_width && y2 >= 0 && y2 < rotated_height) {
            set_px_rotm90(frame, width, height, x2, y2, color);
        }
    }
}

line_control_point_t sort_line(uint16_t *frame, int width, int height)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int band_start_row = rotated_height - CONTROL_BAND_HEIGHT;
    long sum_x = 0;
    long sum_y = 0;
    long sum_width = 0;
    long sum_start_x = 0;
    long sum_end_x = 0;
    long sum_yy = 0;
    long sum_xy = 0;
    int center_count = 0;
    int width_count = 0;
    line_control_point_t output = { false, -1, -1, 0, -1, -1, 0, 0, 0, 0, 0 };

    for (int row = band_start_row; row < rotated_height; ++row) {
        long row_sum_x = 0;
        int row_center_count = 0;
        int row_first_edge_x = -1;
        int row_last_edge_x = -1;

        for (int col = 0; col < rotated_width; ++col) {
            const uint16_t px = get_px_rotm90(frame, width, height, col, row);

            if (px == COLOR_BLUE) {
                if (row_first_edge_x < 0) {
                    row_first_edge_x = col;
                }
                row_last_edge_x = col;
                continue;
            }

            if (px == COLOR_GREEN) {
                row_sum_x += col;
                row_center_count++;
            }
        }

        if (row_center_count > 0 &&
            row_first_edge_x >= 0 &&
            row_last_edge_x >= row_first_edge_x &&
            is_line_segment_width_valid(row_last_edge_x - row_first_edge_x + 1, rotated_width)) {
            sum_x += row_sum_x;
            sum_y += (long)row * row_center_count;
            sum_yy += (long)row * row * row_center_count;
            sum_xy += row_sum_x * row;
            center_count += row_center_count;
            sum_width += row_last_edge_x - row_first_edge_x + 1;
            sum_start_x += row_first_edge_x;
            sum_end_x += row_last_edge_x;
            width_count++;
        }
    }

    if (center_count <= 0) {
        return output;
    }

    output.found = true;
    output.center_x = (int)((sum_x + center_count / 2) / center_count);
    output.center_y = (int)((sum_y + center_count / 2) / center_count);
    {
        const long den = (long)center_count * sum_yy - sum_y * sum_y;
        if (den != 0) {
            const long num = (long)center_count * sum_xy - sum_y * sum_x;
            output.slope_q8 = (int)((num * 256 + den / 2) / den);
            output.slope_q8 = clamp_int(output.slope_q8, -1024, 1024);
            output.start_slope_q8 = output.slope_q8;
            output.end_slope_q8 = output.slope_q8;
        }
    }
    if (width_count > 0) {
        output.width = (int)((sum_width + width_count / 2) / width_count);
        output.start_x = (int)((sum_start_x + width_count / 2) / width_count);
        output.end_x = (int)((sum_end_x + width_count / 2) / width_count);
        output.start_proj_q8 = project_x_q8(output.start_x, output.center_y, output.slope_q8);
        output.end_proj_q8 = project_x_q8(output.end_x, output.center_y, output.slope_q8);
    }

    return output;
}

line_control_point_t refine_line_slope_between(uint16_t *frame,
                                               int width,
                                               int height,
                                               line_control_point_t control_point,
                                               int target_y)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int row_min = clamp_int((target_y < control_point.center_y) ? target_y : control_point.center_y,
                                  0,
                                  rotated_height - 1);
    const int row_max = clamp_int((target_y > control_point.center_y) ? target_y : control_point.center_y,
                                  0,
                                  rotated_height - 1);
    long sum_x = 0;
    long sum_y = 0;
    long sum_yy = 0;
    long sum_xy = 0;
    long sum_start_x = 0;
    long sum_start_y = 0;
    long sum_start_yy = 0;
    long sum_start_xy = 0;
    long sum_end_x = 0;
    long sum_end_y = 0;
    long sum_end_yy = 0;
    long sum_end_xy = 0;
    int center_count = 0;
    int edge_count = 0;

    if (!control_point.found || row_max <= row_min) {
        return control_point;
    }

    for (int row = row_min; row <= row_max; ++row) {
        int first_edge_x = -1;
        int last_edge_x = -1;
        int nearest_left_edge_x = -1;
        int nearest_right_edge_x = -1;
        int row_center_count = 0;
        const int expected_center_x =
            control_point.center_x +
            (control_point.slope_q8 * (row - control_point.center_y) + 128) / 256;

        for (int col = 0; col < rotated_width; ++col) {
            const uint16_t px = get_px_rotm90(frame, width, height, col, row);

            if (px == COLOR_BLUE) {
                if (first_edge_x < 0) {
                    first_edge_x = col;
                }
                last_edge_x = col;
                if (col <= expected_center_x &&
                    (nearest_left_edge_x < 0 || col > nearest_left_edge_x)) {
                    nearest_left_edge_x = col;
                }
                if (col >= expected_center_x &&
                    (nearest_right_edge_x < 0 || col < nearest_right_edge_x)) {
                    nearest_right_edge_x = col;
                }
                continue;
            }

            if (px == COLOR_GREEN) {
                if (iabs_int(col - expected_center_x) > LINE_REFINE_MAX_CENTER_DELTA) {
                    continue;
                }
                sum_x += col;
                sum_y += row;
                sum_yy += (long)row * row;
                sum_xy += (long)col * row;
                center_count++;
                row_center_count++;
            }
        }

        if (row_center_count == 1 &&
            nearest_left_edge_x >= 0 &&
            nearest_right_edge_x >= nearest_left_edge_x) {
            sum_start_x += nearest_left_edge_x;
            sum_start_y += row;
            sum_start_yy += (long)row * row;
            sum_start_xy += (long)nearest_left_edge_x * row;
            sum_end_x += nearest_right_edge_x;
            sum_end_y += row;
            sum_end_yy += (long)row * row;
            sum_end_xy += (long)nearest_right_edge_x * row;
            edge_count++;
        }
    }

    if (center_count >= 4) {
        control_point.slope_q8 = fit_x_from_y_q8(sum_x, sum_y, sum_yy, sum_xy, center_count);
    }

    if (edge_count >= 4) {
        control_point.start_slope_q8 = fit_x_from_y_q8(sum_start_x,
                                                       sum_start_y,
                                                       sum_start_yy,
                                                       sum_start_xy,
                                                       edge_count);
        control_point.end_slope_q8 = fit_x_from_y_q8(sum_end_x,
                                                     sum_end_y,
                                                     sum_end_yy,
                                                     sum_end_xy,
                                                     edge_count);
    } else {
        control_point.start_slope_q8 = control_point.slope_q8;
        control_point.end_slope_q8 = control_point.slope_q8;
    }

    control_point.start_proj_q8 = project_x_q8(control_point.start_x,
                                               control_point.center_y,
                                               control_point.start_slope_q8);
    control_point.end_proj_q8 = project_x_q8(control_point.end_x,
                                             control_point.center_y,
                                             control_point.end_slope_q8);

    return control_point;
}
