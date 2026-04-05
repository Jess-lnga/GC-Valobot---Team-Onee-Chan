#ifndef FRAME_ANALYSIS_HELPERS_H
#define FRAME_ANALYSIS_HELPERS_H

#include <stdbool.h>
#include <stdint.h>

#define COLOR_BLACK  0x0000
#define COLOR_WHITE  0xFFFF
#define COLOR_GREEN  0x07E0
#define COLOR_BLUE   0x001F
#define COLOR_PURPLE 0xF81F

typedef struct {
    bool found;
    int center_x;
    int center_y;
} line_control_point_t;

void filter_black_pxl(uint16_t *frame, int width, int height);
void find_black_segments(uint16_t *frame, int width, int height);
line_control_point_t sort_line(uint16_t *frame, int width, int height);

#endif
