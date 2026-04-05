#include "frame_analysis.h"

#include <stdbool.h>
#include <stdint.h>

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF

#define BRIGHTNESS_HARD_BLACK_MAX     26
#define BRIGHTNESS_DYNAMIC_MARGIN      8
#define CHROMA_BLACK_MAX              10
#define CHROMA_NEAR_BLACK_MAX         16
#define SATURATION_BLACK_LIMIT        14
#define MIN_DYNAMIC_BLACK_THRESHOLD   24
#define MAX_DYNAMIC_BLACK_THRESHOLD   60

static volatile int g_line_pos = 60;
static volatile int g_line_found = 0;
static volatile int g_line_measurement_valid = 0;
static volatile int g_line_lost_frames = 0;
static volatile int g_last_seen_side = 0;

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

static void filter_black_pxl(uint16_t *frame, int width, int height)
{
    const int black_threshold = compute_dynamic_black_threshold(frame, width, height);
    const int pixel_count = width * height;

    for (int i = 0; i < pixel_count; ++i) {
        frame[i] = is_black_pixel(frame[i], black_threshold) ? COLOR_BLACK : COLOR_WHITE;
    }
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

    g_line_found = 0;
    g_line_measurement_valid = 0;
    g_line_lost_frames++;

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
