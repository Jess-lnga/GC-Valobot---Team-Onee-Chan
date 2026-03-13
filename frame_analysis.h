#ifndef FRAME_ANALYSIS_H
#define FRAME_ANALYSIS_H

#include <stdint.h>

typedef struct {
    int found;          // 1 si ligne valide pour le contrôle, 0 sinon
    int row;            // y dans la vue rotée
    int left_x;         // x gauche dans la vue rotée
    int right_x;        // x droite dans la vue rotée
    int center_x;       // centre retenu dans la vue rotée

    int used_points;    // nombre de points utilisés pour la moyenne
    int first_used_y;   // y du premier point utilisé (le plus bas)
    int threshold;      // seuil noir calculé dynamiquement
} line_detection_t;

// Analyse la frame, met à jour l'état global de suivi, et optionnellement
// dessine des points de debug dans l'image.
void analyze_line_and_update_state(uint16_t *frame, int width, int height, line_detection_t *result);

// Accesseurs pour le contrôleur
int  get_line_pos(void);         // retourne la dernière position valide
int  is_line_found(void);        // 1 si la ligne est actuellement trouvée
int  get_last_seen_side(void);   // -1 gauche, +1 droite, 0 inconnu

#endif