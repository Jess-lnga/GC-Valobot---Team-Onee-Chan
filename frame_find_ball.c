#include "frame_find_ball.h"
#include <math.h>
#include <stddef.h>

static ball_detection_t g_ball;

static bool is_red(uint16_t pixel) {
    int r5 = (pixel >> 11) & 31;
    int g6 = (pixel >> 5) & 63;
    int b5 = pixel & 31;
    // Comparer les trois composantes sur la meme echelle 0..255.
    int r = (r5 << 3) | (r5 >> 2);
    int g = (g6 << 2) | (g6 >> 4);
    int b = (b5 << 3) | (b5 >> 2);
    return r >= BALL_RED_MIN && r - g >= BALL_RED_MARGIN &&
           r - b >= BALL_RED_MARGIN &&
           r * 100 >= g * BALL_RED_GREEN_PERCENT &&
           r * 100 >= b * BALL_RED_BLUE_PERCENT;
}

static void clear_detection(void) {
    g_ball = (ball_detection_t){0};
    g_ball.center_x = g_ball.center_y = -1.0f;
    g_ball.min_x = g_ball.min_y = g_ball.max_x = g_ball.max_y = -1;
}

uint16_t *find_ball(uint16_t *frame, int width, int height) {
    clear_detection();
    // La limite garantit aussi l'absence de debordement des sommes uint64_t.
    if (!frame || width <= 0 || height <= 0 || width > 65535 || height > 65535 ||
        (size_t)width > SIZE_MAX / (size_t)height / sizeof(*frame)) return frame;

    float lo_x = 0, hi_x = (float)(width - 1);
    float lo_y = 0, hi_y = (float)(height - 1);
    ball_detection_t result = {0};

    // Premier passage : statistiques de tous les pixels rouges.
    // Puis deux rejets successifs a +/- k sigma, separement sur x et y.
    // Les bornes se resserrent seulement : un pixel rejete ne revient pas.
    for (int pass = 0; pass <= BALL_SIGMA_PASSES; ++pass) {
        uint64_t sx = 0, sy = 0, sxx = 0, syy = 0;
        uint32_t count = 0;
        int xmin = width, xmax = -1, ymin = height, ymax = -1;
        // Arrondir les bornes une seule fois, pas de flottants par pixel.
        int x0 = (int)ceilf(lo_x), x1 = (int)floorf(hi_x);
        int y0 = (int)ceilf(lo_y), y1 = (int)floorf(hi_y);
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                if (!is_red(frame[(size_t)y * (size_t)width + (size_t)x])) continue;
                ++count;
                sx += (uint32_t)x; sy += (uint32_t)y;
                sxx += (uint64_t)x * (uint64_t)x;
                syy += (uint64_t)y * (uint64_t)y;
                if (x < xmin) xmin = x;
                if (x > xmax) xmax = x;
                if (y < ymin) ymin = y;
                if (y > ymax) ymax = y;
            }
        }
        if (pass == 0) g_ball.red_pixel_count = count;
        g_ball.pixel_count = count;
        if (count < BALL_MIN_RED_PIXELS) return frame;

        result.center_x = (float)sx / (float)count;
        result.center_y = (float)sy / (float)count;
        float vx = (float)sxx / (float)count - result.center_x * result.center_x;
        float vy = (float)syy / (float)count - result.center_y * result.center_y;
        result.stddev_x = sqrtf(fmaxf(0, vx));
        result.stddev_y = sqrtf(fmaxf(0, vy));
        result.min_x = xmin; result.max_x = xmax;
        result.min_y = ymin; result.max_y = ymax;

        // Rayon minimal de 1 px pour les petits objets ou un axe sans variance.
        float rx = fmaxf(1.0f, BALL_SIGMA_FACTOR * result.stddev_x);
        float ry = fmaxf(1.0f, BALL_SIGMA_FACTOR * result.stddev_y);
        lo_x = fmaxf(lo_x, result.center_x - rx);
        hi_x = fminf(hi_x, result.center_x + rx);
        lo_y = fmaxf(lo_y, result.center_y - ry);
        hi_y = fminf(hi_y, result.center_y + ry);
    }

    result.found = true;
    result.red_pixel_count = g_ball.red_pixel_count;
    result.pixel_count = g_ball.pixel_count;
    g_ball = result;

    // Marge de 1 pixel autour des limites retenues, bornee a l'image.
    int left = result.min_x > 0 ? result.min_x - 1 : 0;
    int right = result.max_x < width - 1 ? result.max_x + 1 : width - 1;
    int top = result.min_y > 0 ? result.min_y - 1 : 0;
    int bottom = result.max_y < height - 1 ? result.max_y + 1 : height - 1;
    for (int x = left; x <= right; ++x) {
        frame[(size_t)top * width + x] = BALL_BOX_COLOR;
        frame[(size_t)bottom * width + x] = BALL_BOX_COLOR;
    }
    for (int y = top; y <= bottom; ++y) {
        frame[(size_t)y * width + left] = BALL_BOX_COLOR;
        frame[(size_t)y * width + right] = BALL_BOX_COLOR;
    }
    return frame;
}

bool is_ball_found(void) { return g_ball.found; }
ball_detection_t get_ball_detection(void) { return g_ball; }
