#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../frame_find_ball.c"

#define W 160
#define H 120
static uint16_t frame[W * H];
static uint16_t original[W * H];
static void block(int x0, int y0, int width, int height, uint16_t color) {
    for (int y = y0; y < y0 + height; ++y)
        for (int x = x0; x < x0 + width; ++x) frame[y * W + x] = color;
}
static void blank(void) { memset(frame, 0, sizeof(frame)); }

int main(void) {
    blank();
    block(20, 20, 10, 10, 0xF800);
    frame[0] = frame[W * H - 1] = 0xF800;
    memcpy(original, frame, sizeof(frame));
    assert(find_ball(frame, W, H) == frame);
    ball_detection_t b = get_ball_detection();
    assert(is_ball_found() && b.red_pixel_count == 102 && b.pixel_count == 100);
    assert(fabsf(b.center_x - 24.5f) < 0.01f && fabsf(b.center_y - 24.5f) < 0.01f);
    assert(fabsf(b.stddev_x - sqrtf(8.25f)) < 0.01f);
    assert(b.min_x == 20 && b.max_x == 29 && b.min_y == 20 && b.max_y == 29);
    assert(frame[19 * W + 19] == BALL_BOX_COLOR);
    assert(frame[30 * W + 30] == BALL_BOX_COLOR);
    assert(frame[24 * W + 24] == 0xF800); // Couleur de la balle preservee.
    assert(frame[0] == 0xF800); // Le bruit est exclu du calcul, pas efface de l'image.
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
        bool border = ((y == 19 || y == 30) && x >= 19 && x <= 30) ||
                      ((x == 19 || x == 30) && y >= 19 && y <= 30);
        if (!border) assert(frame[y * W + x] == original[y * W + x]);
    }

    // Fond clair et couleurs non rouges : aucun cadre, ancien resultat efface.
    block(0, 0, W, H, 0xFFFF);
    block(0, 0, 30, 30, 0x8410); // Gris RGB565, malgre les 6 bits verts.
    block(30, 0, 30, 30, 0x07E0);
    block(60, 0, 30, 30, 0x001F);
    memcpy(original, frame, sizeof(frame));
    find_ball(frame, W, H);
    assert(!is_ball_found() && get_ball_detection().center_x == -1);
    assert(memcmp(original, frame, sizeof(frame)) == 0);

    // Seuil exact, rouge sombre et variance nulle sur un axe.
    blank(); block(20, 20, 20, 1, 0x4000);
    find_ball(frame, W, H);
    b = get_ball_detection();
    assert(b.found && b.pixel_count == 20 && b.stddev_y == 0);
    blank(); block(20, 20, 19, 1, 0xF800);
    memcpy(original, frame, sizeof(frame));
    find_ball(frame, W, H);
    assert(!is_ball_found() && memcmp(original, frame, sizeof(frame)) == 0);

    // Assez de rouge avant rejet, pas assez apres : aucun dessin.
    frame[0] = frame[W * H - 1] = 0xF800;
    memcpy(original, frame, sizeof(frame));
    find_ball(frame, W, H);
    assert(!is_ball_found() && get_ball_detection().red_pixel_count == 21);
    assert(memcmp(original, frame, sizeof(frame)) == 0);

    blank(); block(0, 0, 5, 5, 0xF800);
    find_ball(frame, W, H);
    assert(is_ball_found() && get_ball_detection().min_x == 0);
    assert(frame[0] == BALL_BOX_COLOR && frame[5 * W + 5] == BALL_BOX_COLOR);
    assert(find_ball(NULL, W, H) == NULL && !is_ball_found());
    assert(find_ball(frame, 0, H) == frame && !is_ball_found());
    find_ball(frame, 65536, H); assert(!is_ball_found());
    puts("Detection balle : tous les scenarios passent.");
    return 0;
}
