#include "line_following.h"
#include "frame_analysis.h"
#include <stdio.h>
#include <math.h>

#define CENTER 60.0f

/////////////////////////////////////////////////////////////
// ==================== SUIVI DE LIGNE =====================
/////////////////////////////////////////////////////////////

#define LINE_CENTER_PIXEL     60.0f   // pour image rotée de largeur 120
#define FORWARD_SPEED_CMD     3.0f

// PI en mode suivi
static float K_P = 0.1f;
static float K_I = 0.002f;

#define ABS_ERROR_I_MAX       300.0f
#define ABS_ANGLE_MAX_DEG      25.0f

// Recherche quand ligne perdue
#define SEARCH_ANGLE_DEG        12.0f

typedef enum {
    LINE_MODE_SEARCH = 0,
    LINE_MODE_TRACK  = 1
} line_mode_t;

static line_mode_t g_mode = LINE_MODE_SEARCH;

static float g_error = 0.0f;
static float g_error_I = 0.0f;
static float g_angle_deg = 0.0f;

void follow_line_reset(void)
{
    g_error = 0.0f;
    g_error_I = 0.0f;
    g_angle_deg = 0.0f;
}

void follow_line_step(void)
{
    const int found = is_line_found();

    if (found) {

        if (g_mode != LINE_MODE_TRACK) {
            follow_line_reset();
            g_mode = LINE_MODE_TRACK;
        }

        const float line_pos = (float)get_line_pos();
        g_error = line_pos - LINE_CENTER_PIXEL;

        g_error_I += g_error;
        if (g_error_I >  ABS_ERROR_I_MAX) g_error_I =  ABS_ERROR_I_MAX;
        if (g_error_I < -ABS_ERROR_I_MAX) g_error_I = -ABS_ERROR_I_MAX;

        g_angle_deg = K_P * g_error + K_I * g_error_I;

        if (g_angle_deg >  ABS_ANGLE_MAX_DEG) g_angle_deg =  ABS_ANGLE_MAX_DEG;
        if (g_angle_deg < -ABS_ANGLE_MAX_DEG) g_angle_deg = -ABS_ANGLE_MAX_DEG;

        //printf("[TRACK] line=%0.2f err=%0.2f I=%0.2f angle_deg=%0.2f\n",
               //line_pos, g_error, g_error_I, g_angle_deg);

        move(FORWARD_SPEED_CMD, 0, g_angle_deg * (float)M_PI / 180.0f);
        return;
    }

    // Ligne perdue
    if (g_mode != LINE_MODE_SEARCH) {
        follow_line_reset();
        g_mode = LINE_MODE_SEARCH;
    }

    int side = get_last_seen_side();
    if (side == 0) {
        side = +1; // choix par défaut si jamais on n'a encore rien vu
    }

    g_angle_deg = side * SEARCH_ANGLE_DEG;

    //printf("[SEARCH] last_side=%d search_angle_deg=%0.2f\n",
           //side, g_angle_deg);

    // 0 translation, rotation pure
    move(0.0f, 0, g_angle_deg * (float)M_PI / 180.0f);
}

/////////////////////////////////////////////////////////////
// ================= ALIGNEMENT SUR CIBLE ==================
/////////////////////////////////////////////////////////////

#define TARGET_CENTER_PIXEL         60.0f

// Position neutre et limites mécaniques globales
#define TARGET_NEUTRAL_US         1500.0f
#define TARGET_MIN_US             1000.0f
#define TARGET_MAX_US             2000.0f

// Zone utile avant recentrage du corps
#define TARGET_SOFT_MIN_US        1200.0f
#define TARGET_SOFT_MAX_US        1800.0f



// Petite zone morte pour éviter de gigoter
#define TARGET_DEADBAND_PX          3.0f

static float g_target_error = 0.0f;
static float g_target_rotation_us = TARGET_NEUTRAL_US;

static float clamp_float(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static float absf_local(float x)
{
    return (x < 0.0f) ? -x : x;
}

void target_align_reset(void)
{
    g_target_error = 0.0f;
    g_target_rotation_us = TARGET_NEUTRAL_US;
}


static int previous_angle_us = 1500;

// Contrôleur proportionnel simple
static float TARGET_K_P = 5.0f;   // us / pixel


void target_align_step(void)
{
    //const int found = is_target_found();

    // Si cible perdue : on regarde droit devant et on ne fait rien d'autre
    //if (!found) {
    //    recenter(previous_angle_us);
    //    //turn_without_moving((int)TARGET_NEUTRAL_US);
    //    return;
    //}

    const float target_x = (float)get_target_pos_y();

    // Erreur horizontale seulement
    g_target_error = target_x - TARGET_CENTER_PIXEL;

    // Contrôle proportionnel simple
    previous_angle_us = 1500 -  TARGET_K_P * g_target_error;

    if((previous_angle_us <= 1200)||(previous_angle_us >= 1800)){
        recenter(previous_angle_us);
        previous_angle_us = 1500;
    }
    
    turn_without_moving(previous_angle_us);


}