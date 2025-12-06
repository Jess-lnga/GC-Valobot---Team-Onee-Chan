#include "frame_analysis.h"
#include <stdbool.h>

// Quelques couleurs utiles en RGB565
#define COLOR_RED    0xF800
#define COLOR_GREEN  0x07E0
#define COLOR_BLACK  0x0000
#define COLOR_WHITE  0xFFFF

// Largeur minimale d'un segment pour être considéré comme une "vraie ligne"
#define MIN_SEGMENT_WIDTH  8

// Luminosité max en mode R5+G6+B5 : 31 + 63 + 31 = 125
#define MAX_BRIGHTNESS     125

// Nombre max de segments bruts par ligne (suffisant pour 160 px de large)
#define MAX_SEGMENTS_PER_LINE  32

// Quand on fusionne les segments, tolérance en pixels pour combiner deux
// segments très proches (petit trou de 1 px par exemple)
#define MERGE_GAP_TOLERANCE    1

// Seuil de chroma pour considérer un pixel comme "neutre" (pas trop coloré).
// Chroma = max(R5,G6,B5) - min(R5,G6,B5).
// 0 = parfaitement gris/noir, plus c'est grand plus la couleur est saturée.
#define CHROMA_BLACK_MAX       6   // à ajuster si besoin

// -----------------------------------------------------------------------------
// Extraction des composantes depuis RGB565
// -----------------------------------------------------------------------------
static inline void rgb565_to_components(uint16_t px,
                                        uint8_t *r5,
                                        uint8_t *g6,
                                        uint8_t *b5)
{
    *r5 = (px >> 11) & 0x1F;  // 5 bits
    *g6 = (px >> 5)  & 0x3F;  // 6 bits
    *b5 =  px        & 0x1F;  // 5 bits
}

// Retourne :
//  - brightness : R+G+B (0..125)
//  - chroma     : max(R,G,B) - min(R,G,B), mesure de saturation/couleur
static inline void pixel_brightness_chroma(uint16_t px,
                                           uint8_t *brightness,
                                           uint8_t *chroma)
{
    uint8_t r5, g6, b5;
    rgb565_to_components(px, &r5, &g6, &b5);

    uint8_t maxv = r5;
    if (g6 > maxv) maxv = g6;
    if (b5 > maxv) maxv = b5;

    uint8_t minv = r5;
    if (g6 < minv) minv = g6;
    if (b5 < minv) minv = b5;

    *brightness = (uint8_t)(r5 + g6 + b5);   // 0..125
    *chroma     = (uint8_t)(maxv - minv);    // 0..63
}

// Version simplifiée quand on n'a besoin que de la luminosité
static inline int pixel_brightness(uint16_t px)
{
    uint8_t b, c;
    pixel_brightness_chroma(px, &b, &c);
    return (int)b;
}

// Test "noir" basé sur :
//  - un seuil dynamique de luminosité (brightness <= threshold)
//  - une chroma faible (pixel peu coloré, proche d'un gris/noir)
static inline bool is_black_with_threshold(uint16_t px, int threshold)
{
    uint8_t b, c;
    pixel_brightness_chroma(px, &b, &c);

    if (b > threshold) {
        return false;
    }

    if (c > CHROMA_BLACK_MAX) {
        // pixel sombre mais très coloré -> on le rejette
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
// Calcule un seuil de "noir" dynamique en fonction des pixels les plus sombres.
// - Histogramme des luminosités 0..125.
// - On prend les N pixels les plus sombres (N <= 1000, ou moins si image petite).
// - On trouve la luminosité b0 telle que cum_hist(b0) ≈ N.
// - On définit threshold = b0 + marge.
// -----------------------------------------------------------------------------
static int compute_dynamic_black_threshold(const uint16_t *frame,
                                           int width,
                                           int height)
{
    int hist[MAX_BRIGHTNESS + 1];
    for (int i = 0; i <= MAX_BRIGHTNESS; ++i) {
        hist[i] = 0;
    }

    const int total_pixels = width * height;

    // 1) Construire l'histogramme des luminosités
    for (int i = 0; i < total_pixels; ++i) {
        int br = pixel_brightness(frame[i]);
        if (br < 0) br = 0;
        if (br > MAX_BRIGHTNESS) br = MAX_BRIGHTNESS;
        hist[br]++;
    }

    // 2) Nombre de pixels "sombres" à considérer
    int target_dark_count = total_pixels;
    if (target_dark_count > 1000) {
        target_dark_count = 1000;
    }
    if (target_dark_count < 100) {
        target_dark_count = total_pixels; // petites images -> on prend tout
    }

    // 3) Trouver la luminosité b0 des N pixels les plus sombres
    int accumulated = 0;
    int b0 = 0;

    for (int b = 0; b <= MAX_BRIGHTNESS; ++b) {
        accumulated += hist[b];
        if (accumulated >= target_dark_count) {
            b0 = b;
            break;
        }
    }

    // 4) Définir un seuil à partir de b0 avec une petite marge
    int threshold = b0 + 3;  // marge de 3 niveaux (0..125)

    if (threshold > MAX_BRIGHTNESS) {
        threshold = MAX_BRIGHTNESS;
    }
    if (threshold < 2) {
        threshold = 2;
    }

    return threshold;
}

// -----------------------------------------------------------------------------
// Analyse d'une frame et marquage de la ligne (bord gauche/droit en rouge,
// centre en vert) sur TOUTES les lignes où une ou plusieurs bandes "noires"
// sont détectées.
//
// - Seuil de noir adaptatif basé sur les pixels les plus sombres de l'image.
// - Un pixel est "noir" s'il est suffisamment sombre ET peu coloré.
// - Sur chaque ligne :
//      1) on détecte tous les segments bruts de pixels noirs,
//      2) on fusionne ceux qui se chevauchent / sont très proches,
//      3) on ne dessine que les segments fusionnés :
//           * bord gauche en rouge
//           * bord droit en rouge
//           * centre en vert
//
// - Si 'result' != NULL :
//      - result->found = 1 si AU MOINS un segment a été trouvé sur l'image
//      - result->row, left_x, right_x, center_x décrivent le premier segment
//        trouvé en partant du bas (le plus proche du robot).
// -----------------------------------------------------------------------------
void frame_analyze_line_rgb565(uint16_t *frame,
                               int width,
                               int height,
                               line_detection_t *result)
{
    if (!frame || width <= 0 || height <= 0) {
        if (result) {
            result->found    = 0;
            result->row      = -1;
            result->left_x   = -1;
            result->right_x  = -1;
            result->center_x = -1;
        }
        return;
    }

    // Calcul du seuil de "noir" basé sur les pixels les plus sombres
    int black_threshold = compute_dynamic_black_threshold(frame, width, height);

    bool first_found = false;

    if (result) {
        result->found    = 0;
        result->row      = -1;
        result->left_x   = -1;
        result->right_x  = -1;
        result->center_x = -1;
    }

    // Parcours de l'image de bas en haut
    for (int y = height - 1; y >= 0; --y) {

        // ---------------------------------------------------------------------
        // 1) Détection de tous les segments "bruts" sur cette ligne
        // ---------------------------------------------------------------------
        int raw_start[MAX_SEGMENTS_PER_LINE];
        int raw_end  [MAX_SEGMENTS_PER_LINE];
        int raw_count = 0;

        int x = 0;
        while (x < width) {
            // Cherche le début d'un segment "noir"
            while (x < width &&
                   !is_black_with_threshold(frame[y * width + x], black_threshold)) {
                x++;
            }

            if (x >= width) {
                // Plus de pixel noir sur cette ligne
                break;
            }

            int seg_start = x;

            // Avance tant que les pixels restent noirs
            while (x < width &&
                   is_black_with_threshold(frame[y * width + x], black_threshold)) {
                x++;
            }

            int seg_end   = x - 1;
            int seg_width = seg_end - seg_start + 1;

            // Filtre brut : ignore les segments trop petits (bruit)
            if (seg_width >= MIN_SEGMENT_WIDTH && raw_count < MAX_SEGMENTS_PER_LINE) {
                raw_start[raw_count] = seg_start;
                raw_end  [raw_count] = seg_end;
                raw_count++;
            }

            // On continue la recherche à partir de x
        }

        if (raw_count == 0) {
            // pas de segment significatif sur cette ligne
            continue;
        }

        // ---------------------------------------------------------------------
        // 2) Fusion des segments qui se chevauchent ou sont très proches
        // ---------------------------------------------------------------------
        int merged_start[MAX_SEGMENTS_PER_LINE];
        int merged_end  [MAX_SEGMENTS_PER_LINE];
        int merged_count = 0;

        for (int i = 0; i < raw_count; ++i) {
            int s = raw_start[i];
            int e = raw_end  [i];

            if (merged_count == 0) {
                merged_start[0] = s;
                merged_end  [0] = e;
                merged_count    = 1;
            } else {
                int last_idx = merged_count - 1;
                int last_s   = merged_start[last_idx];
                int last_e   = merged_end  [last_idx];

                // Chevauchement ou petit gap → fusion
                if (s <= last_e + MERGE_GAP_TOLERANCE) {
                    if (e > last_e) {
                        merged_end[last_idx] = e;
                    }
                    // début reste last_s
                } else {
                    // Segment distinct, on l'ajoute
                    if (merged_count < MAX_SEGMENTS_PER_LINE) {
                        merged_start[merged_count] = s;
                        merged_end  [merged_count] = e;
                        merged_count++;
                    }
                }
            }
        }

        // ---------------------------------------------------------------------
        // 3) Marquage des segments fusionnés + mise à jour éventuelle du résultat
        // ---------------------------------------------------------------------
        for (int i = 0; i < merged_count; ++i) {
            int s = merged_start[i];
            int e = merged_end  [i];
            int w = e - s + 1;

            if (w < MIN_SEGMENT_WIDTH) {
                continue;
            }

            int center = (s + e) / 2;

            // Marquer les bords et le centre dans l'image
            frame[y * width + s]      = COLOR_RED;
            frame[y * width + e]      = COLOR_RED;
            frame[y * width + center] = COLOR_GREEN;

            // Mettre à jour le résultat logique seulement pour
            // le premier segment trouvé en partant du bas.
            if (result && !first_found) {
                result->found    = 1;
                result->row      = y;
                result->left_x   = s;
                result->right_x  = e;
                result->center_x = center;
                first_found      = true;
            }
        }
    }
}
