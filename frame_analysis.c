#include "frame_analysis.h"
#include <stdbool.h>
#include <stdint.h>

// Quelques couleurs utiles en RGB565
#define COLOR_RED    0xF800
#define COLOR_GREEN  0x07E0
#define COLOR_BLACK  0x0000
#define COLOR_WHITE  0xFFFF

// Largeur minimale d'un segment pour être considéré comme une "vraie ligne"
#define MIN_SEGMENT_WIDTH      3

// Luminosité max en mode R5+G6/B5 : 31 + 63 + 31 = 125
#define MAX_BRIGHTNESS         125

// Nombre max de segments bruts par ligne (suffisant pour 160 px de large)
#define MAX_SEGMENTS_PER_LINE  32

// Tolérance en pixels pour fusionner deux segments proches (trou de 1 px, etc.)
#define MERGE_GAP_TOLERANCE    1

// Seuil de chroma pour considérer un pixel comme "neutre" (pas trop coloré).
// Chroma = max(R,G,B) - min(R,G,B).
#define CHROMA_BLACK_MAX       6   // à tuner si besoin

// Nombre max de lignes où on garde un sample (>= hauteur max de ton image)
#define MAX_LINE_SAMPLES       200

// Continuité : écart horizontal max entre centres successifs pour rester
// dans la même "ligne" (courbe continue).
#define MAX_CENTER_STEP        20   // pixels, à tuner selon ton cas

// Nombre minimal de points dans la chaîne continue pour accepter une ligne
#define MIN_CHAIN_LENGTH       5

// Lissage exponentiel des centres
#define SMOOTH_ALPHA           0.6f  // 0.6 = centre actuel, 0.4 = historique

// -----------------------------------------------------------------------------
// Rotation -90° (sens horaire) : vue virtuelle (vW=H, vH=W)
// mapping : (xv,yv) -> (xo,yo) = (yv, H-1-xv)
// -----------------------------------------------------------------------------
static inline int rotm90_xo(int xv, int yv) { (void)xv; return yv; }
static inline int rotm90_yo(int xv, int H)  { return (H - 1) - xv; }

static inline uint16_t get_px_rotm90(const uint16_t *frame, int W, int H, int xv, int yv)
{
    int xo = rotm90_xo(xv, yv);
    int yo = rotm90_yo(xv, H);
    return frame[yo * W + xo];
}

static inline void set_px_rotm90(uint16_t *frame, int W, int H, int xv, int yv, uint16_t color)
{
    int xo = rotm90_xo(xv, yv);
    int yo = rotm90_yo(xv, H);
    frame[yo * W + xo] = color;
}

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
//  - chroma     : max(R,G,B) - min(R,G,B)
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

static inline int pixel_brightness(uint16_t px)
{
    uint8_t b, c;
    pixel_brightness_chroma(px, &b, &c);
    return (int)b;
}

// Test "noir" : sombre + peu coloré
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
// Seuil dynamique de "noir" basé sur les pixels les plus sombres
// (pas besoin de rotation : histogramme identique quelle que soit la vue)
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

    // 1) Histogramme des luminosités
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
        target_dark_count = total_pixels;
    }

    // 3) Trouver la luminosité b0 de ces pixels sombres
    int accumulated = 0;
    int b0 = 0;

    for (int b = 0; b <= MAX_BRIGHTNESS; ++b) {
        accumulated += hist[b];
        if (accumulated >= target_dark_count) {
            b0 = b;
            break;
        }
    }

    // 4) Seuil = b0 + petite marge
    int threshold = b0 + 3;

    if (threshold > MAX_BRIGHTNESS) {
        threshold = MAX_BRIGHTNESS;
    }
    if (threshold < 2) {
        threshold = 2;
    }

    return threshold;
}

// -----------------------------------------------------------------------------
// Analyse d'une frame, construit une chaîne continue de centre de ligne,
// la lisse, et dessine la ligne lissée.
//
// Version adaptée : détection faite sur une vue virtuelle rotée de -90°
// -----------------------------------------------------------------------------
void frame_analyze_line_rgb565(uint16_t *frame,
                               int width,   // W (original)
                               int height,  // H (original)
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

    // Vue rotée -90° : dimensions virtuelles
    const int vW = height; // largeur vue rotée
    const int vH = width;  // hauteur vue rotée

    // Seuil dynamique pour le "noir"
    int black_threshold = compute_dynamic_black_threshold(frame, width, height);

    // Samples par ligne (au plus un "segment principal" par ligne)
    int sample_y     [MAX_LINE_SAMPLES];
    int sample_left  [MAX_LINE_SAMPLES];
    int sample_right [MAX_LINE_SAMPLES];
    int sample_center[MAX_LINE_SAMPLES];
    int sample_count = 0;

    if (result) {
        result->found    = 0;
        result->row      = -1;
        result->left_x   = -1;
        result->right_x  = -1;
        result->center_x = -1;
    }

    // -------------------------------------------------------------------------
    // PHASE 1 : détection par ligne + fusion de segments (dans la vue rotée)
    // On parcourt yv du bas vers le haut, comme avant.
    // -------------------------------------------------------------------------
    for (int yv = vH - 1; yv >= 0; --yv) {

        int raw_start[MAX_SEGMENTS_PER_LINE];
        int raw_end  [MAX_SEGMENTS_PER_LINE];
        int raw_count = 0;

        int xv = 0;
        while (xv < vW) {
            // chercher début de segment noir
            while (xv < vW &&
                   !is_black_with_threshold(get_px_rotm90(frame, width, height, xv, yv),
                                            black_threshold)) {
                xv++;
            }

            if (xv >= vW) {
                break;
            }

            int seg_start = xv;

            // avancer tant que noir
            while (xv < vW &&
                   is_black_with_threshold(get_px_rotm90(frame, width, height, xv, yv),
                                           black_threshold)) {
                xv++;
            }

            int seg_end   = xv - 1;
            int seg_width = seg_end - seg_start + 1;

            if (seg_width >= MIN_SEGMENT_WIDTH && raw_count < MAX_SEGMENTS_PER_LINE) {
                raw_start[raw_count] = seg_start;
                raw_end  [raw_count] = seg_end;
                raw_count++;
            }
        }

        if (raw_count == 0) {
            continue;
        }

        // Fusion des segments qui se chevauchent / sont très proches
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
                int last_e   = merged_end[last_idx];

                if (s <= last_e + MERGE_GAP_TOLERANCE) {
                    if (e > last_e) {
                        merged_end[last_idx] = e;
                    }
                } else {
                    if (merged_count < MAX_SEGMENTS_PER_LINE) {
                        merged_start[merged_count] = s;
                        merged_end  [merged_count] = e;
                        merged_count++;
                    }
                }
            }
        }

        // Choisir un segment "principal" pour cette ligne (le plus large)
        int best_idx   = -1;
        int best_width = 0;

        for (int i = 0; i < merged_count; ++i) {
            int w = merged_end[i] - merged_start[i] + 1;
            if (w > best_width) {
                best_width = w;
                best_idx   = i;
            }
        }

        if (best_idx < 0) {
            continue;
        }

        if (sample_count < MAX_LINE_SAMPLES) {
            int s = merged_start[best_idx];
            int e = merged_end  [best_idx];
            int c = (s + e) / 2;

            sample_y     [sample_count] = yv;
            sample_left  [sample_count] = s;
            sample_right [sample_count] = e;
            sample_center[sample_count] = c;
            sample_count++;
        }
    }

    // Aucun échantillon -> aucune ligne détectée
    if (sample_count == 0) {
        return;
    }

    // -------------------------------------------------------------------------
    // PHASE 2 : construction d'une chaîne continue (courbe) depuis le bas
    // -------------------------------------------------------------------------
    int chain_idx   [MAX_LINE_SAMPLES];
    int chain_count = 0;

    // Point de départ : le plus bas (sample 0)
    chain_idx[chain_count++] = 0;
    int prev_idx = 0;

    for (int i = 1; i < sample_count; ++i) {
        int dy = sample_y[i - 1] - sample_y[i];
        if (dy <= 0) {
            continue;
        }

        int dx = sample_center[i] - sample_center[prev_idx];
        if (dx < 0) dx = -dx;

        if (dx <= MAX_CENTER_STEP) {
            chain_idx[chain_count++] = i;
            prev_idx = i;
        } else {
            break;
        }

        if (chain_count >= MAX_LINE_SAMPLES) {
            break;
        }
    }

    if (chain_count < MIN_CHAIN_LENGTH) {
        return;
    }

    // -------------------------------------------------------------------------
    // PHASE 3 : lissage des centres le long de la chaîne
    // -------------------------------------------------------------------------
    int smooth_center[MAX_LINE_SAMPLES];

    float c_smooth = (float)sample_center[chain_idx[0]];
    smooth_center[chain_idx[0]] = (int)(c_smooth + 0.5f);

    for (int k = 1; k < chain_count; ++k) {
        int idx      = chain_idx[k];
        float c_raw  = (float)sample_center[idx];
        c_smooth     = SMOOTH_ALPHA * c_raw + (1.0f - SMOOTH_ALPHA) * c_smooth;
        smooth_center[idx] = (int)(c_smooth + 0.5f);
    }

    // Largeur moyenne de la ligne sur cette chaîne
    long sum_half_w = 0;
    for (int k = 0; k < chain_count; ++k) {
        int idx = chain_idx[k];
        int w   = sample_right[idx] - sample_left[idx] + 1;
        int hw  = w / 2;
        if (hw < 1) hw = 1;
        sum_half_w += hw;
    }

    int avg_half_w = (int)(sum_half_w / chain_count);
    if (avg_half_w < 1) avg_half_w = 1;

    // -------------------------------------------------------------------------
    // PHASE 4 : dessin de la ligne lissée dans la frame (via SET roté)
    // -------------------------------------------------------------------------
    for (int k = 0; k < chain_count; ++k) {
        int idx = chain_idx[k];
        int yv  = sample_y[idx];
        int c   = smooth_center[idx];

        if (c < 0)      c = 0;
        if (c >= vW)    c = vW - 1;

        int s = c - avg_half_w;
        int e = c + avg_half_w;

        if (s < 0)      s = 0;
        if (e >= vW)    e = vW - 1;

        set_px_rotm90(frame, width, height, s, yv, COLOR_RED);
        set_px_rotm90(frame, width, height, e, yv, COLOR_RED);
        set_px_rotm90(frame, width, height, c, yv, COLOR_GREEN);
    }

    // -------------------------------------------------------------------------
    // PHASE 5 : résultat logique (dans la vue rotée)
    // -------------------------------------------------------------------------
    if (result) {
        int idx0 = chain_idx[0];     // le plus bas de la chaîne
        int y0   = sample_y[idx0];
        int c0   = smooth_center[idx0];

        if (c0 < 0)      c0 = 0;
        if (c0 >= vW)    c0 = vW - 1;

        int s0 = c0 - avg_half_w;
        int e0 = c0 + avg_half_w;

        if (s0 < 0)      s0 = 0;
        if (e0 >= vW)    e0 = vW - 1;

        result->found    = 1;
        result->row      = y0;   // coordonnée Y dans vue rotée (0..vH-1)
        result->left_x   = s0;   // coordonnée X dans vue rotée (0..vW-1)
        result->right_x  = e0;
        result->center_x = c0;
    }
}
