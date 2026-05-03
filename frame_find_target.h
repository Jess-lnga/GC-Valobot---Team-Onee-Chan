// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jerome ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jerome ESSOLA ELANGA

#ifndef FRAME_FIND_TARGET_H
#define FRAME_FIND_TARGET_H

#include <stdbool.h>
#include <stdint.h>

#define FRAME_FIND_TARGET_MAX_TARGETS 6

typedef struct {
    bool found;
    int center_x;
    int center_y;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int width;
    int height;
    int segment_count;
} target_detection_t;

uint16_t *find_targets(uint16_t *frame, int width, int height);

int get_target_count(void);
bool is_target_found(void);
target_detection_t get_target(int index);

#endif
