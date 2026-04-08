// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#ifndef FRAME_ANALYSIS_H
#define FRAME_ANALYSIS_H

#include <stdint.h>

typedef struct {
    int found;
    int center_x;
} line_detection_t;

uint16_t *find_line_pos(uint16_t *frame, int width, int height);

int get_line_pos(void);
int is_line_found(void);
int is_line_measurement_valid(void);
int get_line_lost_frames(void);
int get_last_seen_side(void);

#endif
