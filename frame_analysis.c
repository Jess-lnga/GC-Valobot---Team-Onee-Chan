#include "frame_analysis.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Couleurs debug RGB565
#define COLOR_RED    0xF800
#define COLOR_GREEN  0x07E0
#define COLOR_BLACK  0x0000
#define COLOR_BLUE   0x001F

// Paramètres de détection spatiale
#define MIN_SEGMENT_WIDTH        3
#define MIN_LINE_WIDTH           6
#define MAX_LINE_WIDTH           120

#define MAX_BRIGHTNESS           125
#define MAX_SEGMENTS_PER_LINE    32
#define MERGE_GAP_TOLERANCE      1
#define CHROMA_BLACK_MAX         11

#define MAX_LINE_SAMPLES         200
#define MAX_CENTER_STEP          20

// Contrôle de validité
#define N_POINTS_FOR_AVERAGE     5
#define MIN_POINTS_TO_ACCEPT     3
#define BOTTOM_BAND_HEIGHT       15   // bande basse de l'image rotée

// Robustesse temporelle
#define MAX_POSITION_JUMP          50   // saut max accepté entre deux frames valides
#define POSITION_SMOOTH_ALPHA_NUM   3   // alpha = 3/4
#define POSITION_SMOOTH_ALPHA_DEN   4

// Hystérésis de détection
#define LOST_FRAMES_THRESHOLD      3    // perte réelle après 3 frames invalides
#define FOUND_FRAMES_THRESHOLD     2    // retrouvée après 2 frames valides

// -----------------------------------------------------------------------------
// État global de suivi de ligne
// -----------------------------------------------------------------------------
static int g_line_pos       = 60; // dernière position filtrée publiée
static int g_line_found     = 0;
static int g_last_seen_side = 0;  // -1 gauche, +1 droite, 0 inconnu

// État interne de robustesse temporelle (ligne)
static int g_has_valid_pos           = 0;
static int g_last_valid_raw_center   = 60;
static int g_filtered_center         = 60;

static int g_consecutive_lost_frames  = 0;
static int g_consecutive_found_frames = 0;

int get_line_pos(void) {
    return g_line_pos;
}

int is_line_found(void) {
    return g_line_found;
}

int get_last_seen_side(void) {
    return g_last_seen_side;
}

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
// RGB565 helpers
// -----------------------------------------------------------------------------
static inline void rgb565_to_components(uint16_t px,
                                        uint8_t *r5,
                                        uint8_t *g6,
                                        uint8_t *b5)
{
    *r5 = (px >> 11) & 0x1F;
    *g6 = (px >> 5)  & 0x3F;
    *b5 =  px        & 0x1F;
}

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

    *brightness = (uint8_t)(r5 + g6 + b5);
    *chroma     = (uint8_t)(maxv - minv);
}

static inline int pixel_brightness(uint16_t px)
{
    uint8_t b, c;
    pixel_brightness_chroma(px, &b, &c);
    return (int)b;
}

static inline bool is_black_with_threshold(uint16_t px, int threshold)
{
    uint8_t b, c;
    pixel_brightness_chroma(px, &b, &c);

    if (b > threshold) return false;
    if (c > CHROMA_BLACK_MAX) return false;
    return true;
}

// -----------------------------------------------------------------------------
// Outils simples
// -----------------------------------------------------------------------------
static inline int iabs_int(int x)
{
    return (x < 0) ? -x : x;
}

static inline int clamp_int(int x, int xmin, int xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

// Lissage exponentiel entier :
// filtered = alpha * new + (1-alpha) * old
// ici alpha = POSITION_SMOOTH_ALPHA_NUM / POSITION_SMOOTH_ALPHA_DEN
static int smooth_center_int(int old_value, int new_value)
{
    int num = POSITION_SMOOTH_ALPHA_NUM;
    int den = POSITION_SMOOTH_ALPHA_DEN;

    return (num * new_value + (den - num) * old_value + den / 2) / den;
}

// -----------------------------------------------------------------------------
// Seuil dynamique du noir
// -----------------------------------------------------------------------------
static int compute_dynamic_black_threshold(const uint16_t *frame, int width, int height)
{
    int hist[MAX_BRIGHTNESS + 1];
    for (int i = 0; i <= MAX_BRIGHTNESS; ++i) {
        hist[i] = 0;
    }

    const int total_pixels = width * height;

    for (int i = 0; i < total_pixels; ++i) {
        int br = pixel_brightness(frame[i]);
        if (br < 0) br = 0;
        if (br > MAX_BRIGHTNESS) br = MAX_BRIGHTNESS;
        hist[br]++;
    }

    int target_dark_count = total_pixels;
    if (target_dark_count > 1000) target_dark_count = 1000;
    if (target_dark_count < 100)  target_dark_count = total_pixels;

    int accumulated = 0;
    int b0 = 0;
    for (int b = 0; b <= MAX_BRIGHTNESS; ++b) {
        accumulated += hist[b];
        if (accumulated >= target_dark_count) {
            b0 = b;
            break;
        }
    }

    int threshold = b0 + 3;
    if (threshold > MAX_BRIGHTNESS) threshold = MAX_BRIGHTNESS;
    if (threshold < 2) threshold = 2;

    return threshold;
}

// -----------------------------------------------------------------------------
// Analyse ligne principale
// -----------------------------------------------------------------------------
void analyze_line_and_update_state(uint16_t *frame, int width, int height, line_detection_t *result)
{
    if (result) {
        result->found        = 0;
        result->row          = -1;
        result->left_x       = -1;
        result->right_x      = -1;
        result->center_x     = -1;
        result->used_points  = 0;
        result->first_used_y = -1;
        result->threshold    = -1;
    }

    if (!frame || width <= 0 || height <= 0) {
        g_consecutive_lost_frames++;
        g_consecutive_found_frames = 0;

        if (g_consecutive_lost_frames >= LOST_FRAMES_THRESHOLD) {
            g_line_found = 0;
        }
        return;
    }

    const int vW = height; // largeur vue rotée
    const int vH = width;  // hauteur vue rotée
    const int center_ref = vW / 2;

    const int black_threshold = compute_dynamic_black_threshold(frame, width, height);

    if (result) {
        result->threshold = black_threshold;
    }

    int prev_center = -1;
    bool have_prev = false;

    int used_centers[N_POINTS_FOR_AVERAGE];
    int used_rows   [N_POINTS_FOR_AVERAGE];
    int used_count = 0;

    int best_left_at_bottom = -1;
    int best_right_at_bottom = -1;

    // -------------------------------------------------------------------------
    // Détection spatiale dans la frame courante
    // -------------------------------------------------------------------------
    for (int yv = vH - 1; yv >= 0; --yv) {

        int raw_start[MAX_SEGMENTS_PER_LINE];
        int raw_end  [MAX_SEGMENTS_PER_LINE];
        int raw_count = 0;

        int xv = 0;
        while (xv < vW) {

            while (xv < vW &&
                   !is_black_with_threshold(get_px_rotm90(frame, width, height, xv, yv), black_threshold)) {
                xv++;
            }

            if (xv >= vW) break;

            int s = xv;

            while (xv < vW &&
                   is_black_with_threshold(get_px_rotm90(frame, width, height, xv, yv), black_threshold)) {
                xv++;
            }

            int e = xv - 1;
            int w = e - s + 1;

            if (w >= MIN_SEGMENT_WIDTH && raw_count < MAX_SEGMENTS_PER_LINE) {
                raw_start[raw_count] = s;
                raw_end  [raw_count] = e;
                raw_count++;
            }
        }

        if (raw_count == 0) {
            continue;
        }

        // Fusion des segments proches
        int merged_start[MAX_SEGMENTS_PER_LINE];
        int merged_end  [MAX_SEGMENTS_PER_LINE];
        int merged_count = 0;

        for (int i = 0; i < raw_count; ++i) {
            int s = raw_start[i];
            int e = raw_end[i];

            if (merged_count == 0) {
                merged_start[0] = s;
                merged_end[0] = e;
                merged_count = 1;
            } else {
                int last = merged_count - 1;
                if (s <= merged_end[last] + MERGE_GAP_TOLERANCE) {
                    if (e > merged_end[last]) merged_end[last] = e;
                } else if (merged_count < MAX_SEGMENTS_PER_LINE) {
                    merged_start[merged_count] = s;
                    merged_end[merged_count] = e;
                    merged_count++;
                }
            }
        }

        // Candidats de largeur plausible
        int cand_idx[MAX_SEGMENTS_PER_LINE];
        int cand_count = 0;

        for (int i = 0; i < merged_count; ++i) {
            int w = merged_end[i] - merged_start[i] + 1;
            if (w < MIN_LINE_WIDTH) continue;
            if (w > MAX_LINE_WIDTH) continue;
            cand_idx[cand_count++] = i;
            if (cand_count >= MAX_SEGMENTS_PER_LINE) break;
        }

        if (cand_count == 0) {
            continue;
        }

        // Choix du meilleur segment dans la frame courante
        int best_i = -1;

        if (!have_prev) {
            int best_w = -1;
            for (int k = 0; k < cand_count; ++k) {
                int i = cand_idx[k];
                int w = merged_end[i] - merged_start[i] + 1;
                if (w > best_w) {
                    best_w = w;
                    best_i = i;
                }
            }
        } else {
            int best_dx = 0x7FFFFFFF;
            for (int k = 0; k < cand_count; ++k) {
                int i = cand_idx[k];
                int c = (merged_start[i] + merged_end[i]) / 2;
                int dx = c - prev_center;
                if (dx < 0) dx = -dx;

                if (dx < best_dx) {
                    best_dx = dx;
                    best_i = i;
                }
            }

            if (best_i >= 0) {
                int c = (merged_start[best_i] + merged_end[best_i]) / 2;
                int dx = c - prev_center;
                if (dx < 0) dx = -dx;

                if (dx > MAX_CENTER_STEP) {
                    continue;
                }
            }
        }

        if (best_i < 0) {
            continue;
        }

        int s = merged_start[best_i];
        int e = merged_end[best_i];
        int c = (s + e) / 2;

        // Debug visuel brut
        set_px_rotm90(frame, width, height, c, yv, COLOR_BLUE);

        if (used_count < N_POINTS_FOR_AVERAGE) {
            used_centers[used_count] = c;
            used_rows[used_count]    = yv;
            used_count++;
        }

        if (used_count == 1) {
            best_left_at_bottom  = s;
            best_right_at_bottom = e;
        }

        prev_center = c;
        have_prev = true;
    }

    if (result) {
        result->used_points = used_count;
        if (used_count > 0) {
            result->first_used_y = used_rows[0];
        }
    }

    // -------------------------------------------------------------------------
    // Validation de la frame courante
    // -------------------------------------------------------------------------
    bool frame_valid = true;

    // Critère 1 : assez de points ?
    if (used_count < MIN_POINTS_TO_ACCEPT) {
        frame_valid = false;
    }

    // Critère 2 : présence dans la bande basse ?
    if (frame_valid) {
        bool has_point_in_bottom_band = false;
        const int bottom_band_limit = vH - BOTTOM_BAND_HEIGHT;

        for (int i = 0; i < used_count; ++i) {
            if (used_rows[i] >= bottom_band_limit) {
                has_point_in_bottom_band = true;
                break;
            }
        }

        if (!has_point_in_bottom_band) {
            frame_valid = false;
        }
    }

    int raw_avg_center = -1;

    if (frame_valid) {
        long sum = 0;
        for (int i = 0; i < used_count; ++i) {
            sum += used_centers[i];
        }

        raw_avg_center = (int)((sum + used_count / 2) / used_count);
        raw_avg_center = clamp_int(raw_avg_center, 0, vW - 1);
    }

    // -------------------------------------------------------------------------
    // Rejet des sauts aberrants entre frames
    // -------------------------------------------------------------------------
    bool jump_rejected = false;

    if (frame_valid && g_has_valid_pos) {
        int dx = raw_avg_center - g_last_valid_raw_center;
        if (dx < 0) dx = -dx;

        if (dx > MAX_POSITION_JUMP) {
            frame_valid = false;
            jump_rejected = true;
        }
    }

    // -------------------------------------------------------------------------
    // Hystérésis temporelle : perte / retrouvaille
    // -------------------------------------------------------------------------
    if (frame_valid) {
        g_consecutive_found_frames++;
        g_consecutive_lost_frames = 0;
    } else {
        g_consecutive_lost_frames++;
        g_consecutive_found_frames = 0;
    }

    if (frame_valid) {
        // Initialisation si première vraie mesure
        if (!g_has_valid_pos) {
            g_has_valid_pos = 1;
            g_last_valid_raw_center = raw_avg_center;
            g_filtered_center = raw_avg_center;
        } else {
            g_last_valid_raw_center = raw_avg_center;
            g_filtered_center = smooth_center_int(g_filtered_center, raw_avg_center);
        }

        g_filtered_center = clamp_int(g_filtered_center, 0, vW - 1);
        g_line_pos = g_filtered_center;

        if (g_line_pos < center_ref) {
            g_last_seen_side = -1;
        } else if (g_line_pos > center_ref) {
            g_last_seen_side = +1;
        }

        if (g_line_found) {
            g_line_found = 1;
        } else {
            if (g_consecutive_found_frames >= FOUND_FRAMES_THRESHOLD) {
                g_line_found = 1;
            }
        }
    } else {
        if (g_consecutive_lost_frames >= LOST_FRAMES_THRESHOLD) {
            g_line_found = 0;
        }
    }

    // -------------------------------------------------------------------------
    // Debug visuel de la position filtrée
    // -------------------------------------------------------------------------
    if (frame_valid && used_count > 0) {
        set_px_rotm90(frame, width, height, g_line_pos, used_rows[0], COLOR_RED);
    } else if (jump_rejected && used_count > 0) {
        if (raw_avg_center >= 0 && raw_avg_center < vW) {
            set_px_rotm90(frame, width, height, raw_avg_center, used_rows[0], COLOR_GREEN);
        }
    }

    // -------------------------------------------------------------------------
    // Remplissage du résultat
    // -----------------------------------------------------------------------------
    if (result) {
        result->found    = g_line_found;
        result->center_x = g_line_pos;

        if (frame_valid && used_count > 0) {
            result->row      = used_rows[0];
            result->left_x   = best_left_at_bottom;
            result->right_x  = best_right_at_bottom;
        }
    }
}

// ============================================================================
// ======================= DÉTECTION DE CIBLE BLEUE ===========================
// ============================================================================

// Sous-échantillonnage spatial
#define TARGET_SAMPLE_STEP_X          4
#define TARGET_SAMPLE_STEP_Y          4

// Filtre bleu simple et large
#define TARGET_BLUE_MIN_B5            8
#define TARGET_BLUE_MIN_CHROMA        6
#define TARGET_BLUE_MARGIN_R          4
#define TARGET_BLUE_MARGIN_G          2

// Validation minimale
#define TARGET_MIN_BLUE_POINTS        6

// Robustesse temporelle cible
#define TARGET_MAX_POSITION_JUMP      45
#define TARGET_SMOOTH_ALPHA_NUM        3
#define TARGET_SMOOTH_ALPHA_DEN        4
#define TARGET_LOST_FRAMES_THRESHOLD   3
#define TARGET_FOUND_FRAMES_THRESHOLD  2

// -----------------------------------------------------------------------------
// État global partagé cible
// -----------------------------------------------------------------------------
// volatile : utile ici car accès depuis 2 cores
static volatile int g_target_found = 0;
static volatile int g_target_pos   = 60;   // position utile partagée avec le contrôleur
static volatile int g_target_last_seen_side = 0; // -1, +1, 0

// État interne de robustesse temporelle
static int g_target_has_valid_pos = 0;
static int g_target_last_valid_raw_pos = 60;
static int g_target_filtered_pos = 60;

static int g_target_consecutive_lost_frames = 0;
static int g_target_consecutive_found_frames = 0;

// -----------------------------------------------------------------------------
// Getters
// -----------------------------------------------------------------------------
int get_target_pos(void)
{
    return g_target_pos;
}

// Compatibilité avec ancien code si besoin
int get_target_pos_x(void)
{
    return -1;
}

int get_target_pos_y(void)
{
    return g_target_pos;
}

int is_target_found(void)
{
    return g_target_found;
}

int get_target_last_seen_side(void)
{
    return g_target_last_seen_side;
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static int smooth_target_int(int old_value, int new_value)
{
    int num = TARGET_SMOOTH_ALPHA_NUM;
    int den = TARGET_SMOOTH_ALPHA_DEN;
    return (num * new_value + (den - num) * old_value + den / 2) / den;
}

static inline bool is_blue_target_pixel(uint16_t px)
{
    uint8_t r5, g6, b5;
    rgb565_to_components(px, &r5, &g6, &b5);

    int g5 = (g6 + 1) >> 1; // approx 6 bits -> 5 bits

    int maxv = r5;
    if (g5 > maxv) maxv = g5;
    if (b5 > maxv) maxv = b5;

    int minv = r5;
    if (g5 < minv) minv = g5;
    if (b5 < minv) minv = b5;

    int chroma = maxv - minv;

    if (b5 < TARGET_BLUE_MIN_B5) return false;
    if (chroma < TARGET_BLUE_MIN_CHROMA) return false;
    if (b5 < (int)r5 + TARGET_BLUE_MARGIN_R) return false;
    if (b5 < g5 + TARGET_BLUE_MARGIN_G) return false;

    return true;
}

// -----------------------------------------------------------------------------
// Analyse cible
// -----------------------------------------------------------------------------
void find_target_and_update_state(uint16_t *frame, int width, int height, target_detection_t *result)
{
    if (result) {
        result->found             = 0;
        result->center_x          = -1;
        result->center_y          = -1;
        result->first_pass_points = 0;
        result->used_points       = 0;
        result->std_x             = 0;
        result->std_y             = 0;
        result->bbox_left         = -1;
        result->bbox_right        = -1;
        result->bbox_top          = -1;
        result->bbox_bottom       = -1;
    }

    if (!frame || width <= 0 || height <= 0) {
        g_target_consecutive_lost_frames++;
        g_target_consecutive_found_frames = 0;

        if (g_target_consecutive_lost_frames >= TARGET_LOST_FRAMES_THRESHOLD) {
            g_target_found = 0;
        }
        return;
    }

    const int center_ref = height / 2;

    // -------------------------------------------------------------------------
    // Une seule passe : moyenne de la coordonnée utile
    // Ici on prend Y comme coordonnée de contrôle, car c'est celle que tu utilises
    // -------------------------------------------------------------------------
    int blue_count = 0;
    long sum_pos = 0;

    int bbox_left = width;
    int bbox_right = -1;
    int bbox_top = height;
    int bbox_bottom = -1;

    for (int y = 0; y < height; y += TARGET_SAMPLE_STEP_Y) {
        int row_offset = y * width;

        for (int x = 0; x < width; x += TARGET_SAMPLE_STEP_X) {
            uint16_t px = frame[row_offset + x];

            if (!is_blue_target_pixel(px)) {
                continue;
            }

            blue_count++;
            sum_pos += y;   // <-- coordonnée utile transmise au contrôleur

            if (x < bbox_left)   bbox_left = x;
            if (x > bbox_right)  bbox_right = x;
            if (y < bbox_top)    bbox_top = y;
            if (y > bbox_bottom) bbox_bottom = y;
        }
    }

    if (result) {
        result->first_pass_points = blue_count;
    }

    // -------------------------------------------------------------------------
    // Validation simple
    // -------------------------------------------------------------------------
    bool frame_valid = true;
    int raw_pos = -1;

    if (blue_count < TARGET_MIN_BLUE_POINTS) {
        frame_valid = false;
    } else {
        raw_pos = (int)((sum_pos + blue_count / 2) / blue_count);
        raw_pos = clamp_int(raw_pos, 0, height - 1);
    }

    // -------------------------------------------------------------------------
    // Rejet des sauts aberrants inter-frame
    // -------------------------------------------------------------------------
    bool jump_rejected = false;

    if (frame_valid && g_target_has_valid_pos) {
        int dpos = iabs_int(raw_pos - g_target_last_valid_raw_pos);

        if (dpos > TARGET_MAX_POSITION_JUMP) {
            frame_valid = false;
            jump_rejected = true;
        }
    }

    // -------------------------------------------------------------------------
    // Hystérésis temporelle
    // -------------------------------------------------------------------------
    if (frame_valid) {
        g_target_consecutive_found_frames++;
        g_target_consecutive_lost_frames = 0;
    } else {
        g_target_consecutive_lost_frames++;
        g_target_consecutive_found_frames = 0;
    }

    if (frame_valid) {
        if (!g_target_has_valid_pos) {
            g_target_has_valid_pos = 1;
            g_target_last_valid_raw_pos = raw_pos;
            g_target_filtered_pos = raw_pos;
        } else {
            g_target_last_valid_raw_pos = raw_pos;
            g_target_filtered_pos = smooth_target_int(g_target_filtered_pos, raw_pos);
        }

        g_target_filtered_pos = clamp_int(g_target_filtered_pos, 0, height - 1);
        g_target_pos = g_target_filtered_pos;

        if (g_target_pos < center_ref) {
            g_target_last_seen_side = -1;
        } else if (g_target_pos > center_ref) {
            g_target_last_seen_side = +1;
        }

        if (g_target_found) {
            g_target_found = 1;
        } else if (g_target_consecutive_found_frames >= TARGET_FOUND_FRAMES_THRESHOLD) {
            g_target_found = 1;
        }
    } else {
        if (g_target_consecutive_lost_frames >= TARGET_LOST_FRAMES_THRESHOLD) {
            g_target_found = 0;
        }
    }

    // -------------------------------------------------------------------------
    // Debug visuel
    // -------------------------------------------------------------------------
    if (frame_valid) {
        int debug_x = width / 2;
        frame[g_target_pos * width + debug_x] = COLOR_RED;
    } else if (jump_rejected && raw_pos >= 0) {
        int debug_x = width / 2;
        frame[raw_pos * width + debug_x] = COLOR_GREEN;
    }

    // -------------------------------------------------------------------------
    // Résultat
    // -------------------------------------------------------------------------
    if (result) {
        result->found = g_target_found;
        result->center_x = -1;
        result->center_y = g_target_pos;
        result->used_points = blue_count;
        result->std_x = 0;
        result->std_y = 0;

        if (blue_count > 0) {
            result->bbox_left   = bbox_left;
            result->bbox_right  = bbox_right;
            result->bbox_top    = bbox_top;
            result->bbox_bottom = bbox_bottom;
        }
    }
}