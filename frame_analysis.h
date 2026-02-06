#ifndef FRAME_ANALYSIS_H
#define FRAME_ANALYSIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int found;      // 0 = pas de ligne détectée, 1 = ok
    int row;        // ligne (y) où la ligne a été détectée
    int left_x;     // x du bord gauche de la ligne
    int right_x;    // x du bord droit de la ligne
    int center_x;   // x du centre de la ligne
} line_detection_t;

/**
 * Analyse une frame RGB565 et surligne la ligne noire détectée :
 *  - bord gauche en rouge
 *  - bord droit en rouge
 *  - centre en vert
 *
 * Paramètres :
 *  - frame  : pointeur sur le buffer RGB565 (width * height pixels)
 *  - width  : largeur de l'image
 *  - height : hauteur de l'image
 *  - result : (optionnel) infos sur la ligne détectée
 *
 * Remarque : si result != NULL, result->found vaut 0 si aucune ligne détectée.
 */
void frame_analyze_line_rgb565(uint16_t *frame,
                               int width,
                               int height,
                               line_detection_t *result);

int get_line_pos();

//void find_line(uint16_t *frame, int width, int height);

void find_line(uint16_t *frame, int width, int height, int n_points);

#ifdef __cplusplus
}
#endif

#endif // FRAME_ANALYSIS_H
