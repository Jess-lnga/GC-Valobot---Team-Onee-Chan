#include "frame_analysis_helpers.h"

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

void find_black_segments(uint16_t *frame, int width, int height)
{
    const int rotated_width = height;
    const int rotated_height = width;

    for (int row = 0; row < rotated_height; ++row) {
        int segment_start = -1;
        int last_black_x = -1;
        int white_gap = 0;

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
                    mark_segment(frame, width, height, row, segment_start, last_black_x);
                }
            }

            segment_start = -1;
            last_black_x = -1;
            white_gap = 0;
        }

        if (segment_start >= 0 && last_black_x >= segment_start) {
            const int segment_width = last_black_x - segment_start + 1;
            if (segment_width >= MIN_BLACK_SEGMENT_WIDTH) {
                mark_segment(frame, width, height, row, segment_start, last_black_x);
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
