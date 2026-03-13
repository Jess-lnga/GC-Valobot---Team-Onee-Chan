#ifndef FRAME_ANALYSIS_H
#define FRAME_ANALYSIS_H

#include <stdint.h>

// ============================================================================
// Résultat de détection de ligne
// ============================================================================
typedef struct {
    int found;
    int row;
    int left_x;
    int right_x;
    int center_x;
    int used_points;
    int first_used_y;
    int threshold;
} line_detection_t;

// ============================================================================
// Résultat de détection de cible bleue
// ============================================================================
typedef struct {
    int found;
    int center_x;
    int center_y;
    int first_pass_points;
    int used_points;
    int std_x;
    int std_y;
    int bbox_left;
    int bbox_right;
    int bbox_top;
    int bbox_bottom;
} target_detection_t;

// ============================================================================
// API suivi de ligne
// ============================================================================
int get_line_pos(void);
int is_line_found(void);
int get_last_seen_side(void);

void analyze_line_and_update_state(uint16_t *frame,
                                   int width,
                                   int height,
                                   line_detection_t *result);

// ============================================================================
// API détection / suivi de cible bleue
// ============================================================================
int get_target_pos_x(void);
int get_target_pos_y(void);
int is_target_found(void);
int get_target_last_seen_side(void);

void find_target_and_update_state(uint16_t *frame,
                                  int width,
                                  int height,
                                  target_detection_t *result);

#endif