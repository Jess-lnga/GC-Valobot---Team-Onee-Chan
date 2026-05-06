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
#define T_SHAPE_MIN_WIDTH_RATIO_NUM    1
#define T_SHAPE_MIN_WIDTH_RATIO_DEN    2
#define T_SHAPE_MIN_ABSOLUTE_WIDTH    40
#define T_SHAPE_MIN_ROW_BLACK_RATIO_NUM 2
#define T_SHAPE_MIN_ROW_BLACK_RATIO_DEN 3
#define T_SHAPE_MIN_THICKNESS          4

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
                if (segment_width >= MIN_BLACK_SEGMENT_WIDTH) {
                    register_segment_candidate(segments, &segment_count, segment_start, last_black_x);
                }
            }

            segment_start = -1;
            last_black_x = -1;
            white_gap = 0;
        }

        if (segment_start >= 0 && last_black_x >= segment_start) {
            const int segment_width = last_black_x - segment_start + 1;
            if (segment_width >= MIN_BLACK_SEGMENT_WIDTH) {
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

t_shape_detection_t detect_t_shape(uint16_t *frame, int width, int height)
{
    const int rotated_width = height;
    const int rotated_height = width;
    const int min_wide_span = clamp_int((rotated_width * T_SHAPE_MIN_WIDTH_RATIO_NUM)
                                      / T_SHAPE_MIN_WIDTH_RATIO_DEN,
                                      T_SHAPE_MIN_ABSOLUTE_WIDTH,
                                      rotated_width);
    t_shape_detection_t best = { false, -1, -1, 0, 0 };

    int run_start_row = -1;
    int run_end_row = -1;
    int run_min_x = rotated_width;
    int run_max_x = -1;
    int run_sum_center_x = 0;
    int run_row_count = 0;

    for (int row = 0; row < rotated_height; ++row) {
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
        const bool is_wide_black_row =
            (span_width >= min_wide_span) &&
            (black_count * T_SHAPE_MIN_ROW_BLACK_RATIO_DEN >=
             span_width * T_SHAPE_MIN_ROW_BLACK_RATIO_NUM);

        if (is_wide_black_row) {
            const int center_x = (first_black_x + last_black_x) / 2;

            if (run_start_row < 0) {
                run_start_row = row;
                run_min_x = first_black_x;
                run_max_x = last_black_x;
                run_sum_center_x = 0;
                run_row_count = 0;
            }

            run_end_row = row;
            if (first_black_x < run_min_x) run_min_x = first_black_x;
            if (last_black_x > run_max_x) run_max_x = last_black_x;
            run_sum_center_x += center_x;
            run_row_count++;
            continue;
        }

        if (run_row_count >= T_SHAPE_MIN_THICKNESS &&
            (!best.found || run_row_count > best.thickness)) {
            best.found = true;
            best.center_x = (run_sum_center_x + run_row_count / 2) / run_row_count;
            best.center_y = (run_start_row + run_end_row) / 2;
            best.width = run_max_x - run_min_x + 1;
            best.thickness = run_row_count;
        }

        run_start_row = -1;
        run_end_row = -1;
        run_min_x = rotated_width;
        run_max_x = -1;
        run_sum_center_x = 0;
        run_row_count = 0;
    }

    if (run_row_count >= T_SHAPE_MIN_THICKNESS &&
        (!best.found || run_row_count > best.thickness)) {
        best.found = true;
        best.center_x = (run_sum_center_x + run_row_count / 2) / run_row_count;
        best.center_y = (run_start_row + run_end_row) / 2;
        best.width = run_max_x - run_min_x + 1;
        best.thickness = run_row_count;
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

    for (int dx = -half_bar; dx <= half_bar; ++dx) {
        const int x = x0 + dx;
        if (x >= 0 && x < rotated_width) {
            set_px_rotm90(frame, width, height, x, y0, color);
            if (y0 + 1 < rotated_height) {
                set_px_rotm90(frame, width, height, x, y0 + 1, color);
            }
        }
    }

    for (int dy = 0; dy <= stem_len; ++dy) {
        const int y = y0 + dy;
        if (y >= 0 && y < rotated_height) {
            set_px_rotm90(frame, width, height, x0, y, color);
            if (x0 + 1 < rotated_width) {
                set_px_rotm90(frame, width, height, x0 + 1, y, color);
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
        if (y >= 0 && y < rotated_height) {
            set_px_rotm90(frame, width, height, x0, y, color);
            if (x0 + 1 < rotated_width) {
                set_px_rotm90(frame, width, height, x0 + 1, y, color);
            }
        }
    }

    for (int dx = 0; dx <= branch_len; ++dx) {
        const int x = x0 + dir * dx;
        if (x >= 0 && x < rotated_width) {
            set_px_rotm90(frame, width, height, x, y0, color);
            if (y0 + 1 < rotated_height) {
                set_px_rotm90(frame, width, height, x, y0 + 1, color);
            }
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
    int center_count = 0;
    line_control_point_t output = { false, -1, -1 };

    for (int row = band_start_row; row < rotated_height; ++row) {
        for (int col = 0; col < rotated_width; ++col) {
            if (get_px_rotm90(frame, width, height, col, row) != COLOR_GREEN) {
                continue;
            }

            sum_x += col;
            sum_y += row;
            center_count++;
        }
    }

    if (center_count <= 0) {
        return output;
    }

    output.found = true;
    output.center_x = (int)((sum_x + center_count / 2) / center_count);
    output.center_y = (int)((sum_y + center_count / 2) / center_count);

    return output;
}
