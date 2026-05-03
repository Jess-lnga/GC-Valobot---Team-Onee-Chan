// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jerome ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jerome ESSOLA ELANGA

#ifndef FRAME_FIND_TARGET_HELPERS_H
#define FRAME_FIND_TARGET_HELPERS_H

#include <stdbool.h>
#include <stdint.h>

#include "frame_find_target.h"

#define TARGET_COLOR_BLACK  0x0000
#define TARGET_COLOR_WHITE  0xFFFF
#define TARGET_COLOR_GREEN  0x07E0
#define TARGET_COLOR_BLUE   0x001F
#define TARGET_COLOR_PURPLE 0xF81F
#define TARGET_COLOR_ORANGE 0xFD20
#define TARGET_COLOR_RED    0xF800

typedef struct {
    int start_x;
    int end_x;
    int center_x;
    int width;
    int row;
} target_row_segment_t;

void filter_blue_pxl(uint16_t *frame, int width, int height);
int find_blue_segments_in_row(const uint16_t *frame,
                              int width,
                              int height,
                              int row,
                              target_row_segment_t *segments,
                              int max_segments);
int sort_targets(uint16_t *frame,
                 int width,
                 int height,
                 target_detection_t *targets,
                 int max_targets);
void draw_target(uint16_t *frame,
                 int width,
                 int height,
                 const target_detection_t *target,
                 uint16_t color);

#endif
