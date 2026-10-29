#ifndef FRAME_FIND_BALL_H
#define FRAME_FIND_BALL_H

#include <stdbool.h>
#include <stdint.h>

// Seuils initiaux a calibrer avec les images RGB565 de la camera.
#define BALL_MIN_RED_PIXELS 20u
#define BALL_RED_MIN 40
#define BALL_RED_MARGIN 20
#define BALL_RED_GREEN_PERCENT 170
#define BALL_RED_BLUE_PERCENT 150
#define BALL_SIGMA_FACTOR 2.5f
#define BALL_SIGMA_PASSES 2
#define BALL_BOX_COLOR 0x001Fu // Bleu RGB565

typedef struct {
    bool found;
    uint32_t red_pixel_count;  // Avant rejet statistique.
    uint32_t pixel_count;      // Apres rejet statistique.
    float center_x, center_y;
    float stddev_x, stddev_y;
    int min_x, min_y, max_x, max_y; // Limites inclusives des pixels retenus.
} ball_detection_t;

// Coordonnees de l'image brute : x dans [0,width[, y dans [0,height[.
// Analyse avant dessin : seuls les pixels du cadre bleu sont modifies.
// Retourne frame pour rester compatible avec les autres find_* du projet.
// Appeler sur une nouvelle image, avant les autres filtres/dessins destructifs.
// Resultat global : appeler find_ball et les getters depuis le meme coeur.
uint16_t *find_ball(uint16_t *frame, int width, int height);
bool is_ball_found(void);
ball_detection_t get_ball_detection(void);

#endif
