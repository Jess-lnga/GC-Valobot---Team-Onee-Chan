// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA

#ifndef FRAME_ANALYSIS_HELPERS_H
#define FRAME_ANALYSIS_HELPERS_H

#include <stdbool.h>
#include <stdint.h>

#define COLOR_BLACK  0x0000
#define COLOR_WHITE  0xFFFF
#define COLOR_GREEN  0x07E0
#define COLOR_BLUE   0x001F
#define COLOR_PURPLE 0xF81F
#define COLOR_ORANGE 0xFD20
#define COLOR_RED    0xF800
#define COLOR_YELLOW 0xFFE0

typedef struct {
    bool found;
    int center_x;
    int center_y;
    int width;
    int start_x;
    int end_x;
    int slope_q8;
    int start_slope_q8;
    int end_slope_q8;
    int start_proj_q8;
    int end_proj_q8;
} line_control_point_t;

typedef struct {
    bool found;
    int center_x;
    int center_y;
    int width;
    int thickness;
    int start_x;
    int end_x;
    int slope_q8;
    int band_slope_q8;
    int band_top_slope_q8;
    int band_bottom_slope_q8;
    int band_top_x;
    int band_top_y;
    int band_bottom_x;
    int band_bottom_y;
    int left_ext_x;
    int left_ext_y;
    int right_ext_x;
    int right_ext_y;
    int start_proj_q8;
    int end_proj_q8;
} t_shape_detection_t;

void filter_black_pxl(uint16_t *frame, int width, int height);
void find_black_segments(uint16_t *frame, int width, int height);
line_control_point_t sort_line(uint16_t *frame, int width, int height);
line_control_point_t refine_line_slope_between(uint16_t *frame,
                                               int width,
                                               int height,
                                               line_control_point_t control_point,
                                               int target_y);
void draw_control_point(uint16_t *frame, int width, int height, int center_x, int center_y, uint16_t color);
t_shape_detection_t detect_t_shape(uint16_t *frame,
                                   int width,
                                   int height,
                                   int line_slope_q8,
                                   int nearest_search_row);
void draw_t_shape_marker(uint16_t *frame, int width, int height, t_shape_detection_t t_shape, uint16_t color);
void draw_elbow_marker(uint16_t *frame,
                       int width,
                       int height,
                       int center_x,
                       int center_y,
                       int direction,
                       int slope_q8,
                       uint16_t color);
void draw_detection_geometry(uint16_t *frame,
                             int width,
                             int height,
                             line_control_point_t control_point,
                             t_shape_detection_t t_shape);
void draw_intersection_cross(uint16_t *frame, int width, int height, int center_x, int center_y, uint16_t color);

#endif
