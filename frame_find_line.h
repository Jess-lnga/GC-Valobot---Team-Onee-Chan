// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#ifndef FRAME_ANALYSIS_H
#define FRAME_ANALYSIS_H

#include <stdint.h>

#define LEFT_SIDE -1
#define RIGHT_SIDE 1

typedef struct {
    int found;
    int center_x;
} line_detection_t;

uint16_t *find_line_pos(uint16_t *frame, int width, int height);
uint16_t *find_line_pos_and_detect_t_shape(uint16_t *frame, int width, int height);
//uint16_t *find_line_pos_and_detect_t_shape_2(uint16_t *frame, int width, int height);

int get_line_pos(void);
int is_line_found(void);
int is_line_measurement_valid(void);
int get_line_lost_frames(void);
int get_last_seen_side(void);

int is_t_shape_detected(void);
int get_t_shape_center_x(void);
int get_t_shape_center_y(void);

int is_t_detected(void);
int get_horizontal_t_pos(void);
int get_vertical_t_pos(void);

int is_left_elbow_detected(void);
int get_horizontal_left_elbow_pos(void);
int get_vertical_left_elbow_pos(void);

int is_right_elbow_detected(void);
int get_horizontal_right_elbow_pos(void);
int get_vertical_right_elbow_pos(void);

#endif
