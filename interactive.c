#include "interactive.h"
#include "imu.h"
#include "tof.h"
#include "pca9685.h"
#include "line_following.h"
#include <math.h>

// Reglages a valider sur la mecanique et avec les offsets TOF actuels.
#define TILT_DEG 30.0f
#define LEVEL_DEG 5.0f
#define SELECT_MS 3000
#define LEVEL_MS 500
#define TICK_MS 20
#define MAX_SAMPLE_GAP_MS 200
#define HAND_NEAR_MM 150
#define HAND_FAR_MM 200
#define HAND_LONG_MS 1500
#define TOF_PERIOD_MS 50
#define LEFT_TURN_US 1200
#define RIGHT_TURN_US 1800
#define TOF_LEG_STEP_US 25
#define TOF_TWIST_STEP_US 20
#define HEAD_UP_US SERVO_HEAD_UP_US
#define HEAD_DOWN_US SERVO_HEAD_LINE_US

typedef enum {ACTIVE, FOLD, CENTER, WAIT_LEVEL, STAND} phase_t;
typedef enum {GESTURE_READY, GESTURE_TWIST, GESTURE_RETURN,
              GESTURE_ROTATE, GESTURE_WAIT_RELEASE} gesture_t;
static phase_t phase;
static interactive_mode_t mode, pending;
static int candidate, side, rotation_phase;
static uint64_t candidate_since, level_since, side_since, last_tick, last_tof;
static bool candidate_timing, level_timing, tick_started;
static bool near_left, near_right, near_front;
static bool side_released;
static gesture_t gesture;

static bool hand_near(int mm, bool was_near) {
    // Les getters TOF retournent une valeur negative en cas de mesure invalide.
    return mm >= 0 && mm < (was_near ? HAND_FAR_MM : HAND_NEAR_MM);
}

static int requested_mode(float pitch, float roll) {
    // Une diagonale ambigue ne selectionne aucun mode.
    if (fabsf(pitch) > TILT_DEG && fabsf(roll) > TILT_DEG) return -1;
    if (pitch < -TILT_DEG) return INTERACTIVE_FRONT;
    if (pitch > TILT_DEG) return INTERACTIVE_BACK;
    if (roll < -TILT_DEG) return INTERACTIVE_LEFT;
    if (roll > TILT_DEG) return INTERACTIVE_RIGHT;
    return -1;
}

static bool pose(int coxa, int troc, int head) {
    const int c[4] = {coxa, coxa, coxa, coxa};
    const int t[4] = {troc, troc, troc, troc};
    return servo_pose_step(c, t, head);
}

static bool tof_leg_pose(int coxa, int troc, int step_us) {
    const int c[4] = {coxa, coxa, coxa, coxa};
    const int t[4] = {troc, troc, troc, troc};
    return servo_pose_step_speed(c, t, -1, step_us);
}

void interactive_init(void) {
    phase = ACTIVE;
    mode = pending = INTERACTIVE_RIGHT;
    candidate = -1;
    candidate_timing = level_timing = tick_started = false;
    near_left = near_right = near_front = false;
    side_released = false;
    gesture = GESTURE_READY;
    side = rotation_phase = 0;
    last_tof = 0;
}

static void tof_step(uint64_t now, bool tof_ok) {
    if (tof_ok && now - last_tof >= TOF_PERIOD_MS) {
        mes_all_dist();
        last_tof = now;
        near_left = hand_near(get_dist_left(), near_left);
        near_right = hand_near(get_dist_right(), near_right);
        near_front = hand_near(get_dist_front(), near_front);
    } else if (!tof_ok) {
        near_left = near_right = near_front = false;
    }
    const int head = near_front ? HEAD_DOWN_US : HEAD_UP_US;
    // La tete poursuit sa rampe independamment de la FSM des pattes.
    // Son arrivee en position ne retarde ni la torsion ni le retour au neutre.
    pose(-1, -1, head);
    int detected_side = near_left == near_right ? 0 : (near_left ? -1 : 1);
    int twist = side < 0 ? LEFT_TURN_US : RIGHT_TURN_US;

    switch (gesture) {
    case GESTURE_READY:
        tof_leg_pose(1500, 1500, TOF_LEG_STEP_US);
        if (detected_side) {
            side = detected_side;
            side_since = now;
            side_released = false;
            gesture = GESTURE_TWIST;
        }
        break;
    case GESTURE_TWIST: {
        bool reached = tof_leg_pose(twist, 1500, TOF_TWIST_STEP_US);
        if (detected_side != side) side_released = true;
        // Un geste bref termine sa torsion avant le retour au neutre.
        if (side_released && reached) gesture = GESTURE_RETURN;
        else if (!side_released && reached && now - side_since >= HAND_LONG_MS) {
            rotation_phase = 0;
            gesture = GESTURE_ROTATE;
        }
        break;
    }
    case GESTURE_RETURN:
        if (tof_leg_pose(1500, 1500, TOF_TWIST_STEP_US)) gesture = GESTURE_WAIT_RELEASE;
        break;
    case GESTURE_ROTATE: {
        // Recentrer une diagonale en l'air, la reposer, puis l'autre.
        // Le corps conserve approximativement la rotation acquise par la torsion.
        int c[4] = {twist, twist, twist, twist};
        int t[4] = {1500, 1500, 1500, 1500};
        if (rotation_phase <= 2) {
            if (rotation_phase < 2) t[0] = t[3] = 2000;
            if (rotation_phase >= 1) c[0] = c[3] = 1500;
        } else {
            c[0] = c[3] = 1500;
            if (rotation_phase < 5) t[1] = t[2] = 2000;
            if (rotation_phase >= 4) c[1] = c[2] = 1500;
        }
        if (servo_pose_step_speed(c, t, -1, TOF_LEG_STEP_US) && ++rotation_phase >= 6)
            gesture = GESTURE_WAIT_RELEASE;
        break;
    }
    case GESTURE_WAIT_RELEASE:
        tof_leg_pose(1500, 1500, TOF_LEG_STEP_US);
        if (!near_left && !near_right) gesture = GESTURE_READY;
        break;
    }
}

bool interactive_step(uint64_t now, bool imu_ok, bool tof_ok, bool vision_ready) {
    if (tick_started && now - last_tick < TICK_MS) return false;
    bool gap = tick_started && now - last_tick > MAX_SAMPLE_GAP_MS;
    last_tick = now;
    tick_started = true;

    bool valid = imu_ok && imu_capture();
    float pitch = get_instant_pitch(), roll = get_instant_roll();
    valid = valid && isfinite(pitch) && isfinite(roll);

    if (phase == ACTIVE) {
        int requested = valid ? requested_mode(pitch, roll) : -1;
        if (gap || !valid || requested < 0) {
            candidate_timing = false;
            candidate = -1;
        }
        if (requested >= 0) {
            if (!candidate_timing || candidate != requested) {
                candidate = requested;
                candidate_since = now;
                candidate_timing = true;
            }
            if (now - candidate_since >= SELECT_MS) {
                pending = (interactive_mode_t)candidate;
                phase = FOLD;
                candidate_timing = false;
                level_timing = false;
                gesture = GESTURE_READY;
            }
            // Suspendre les actions des la premiere inclinaison de selection.
            return false;
        }
        // Aucun mouvement autonome avec une mesure IMU invalide ou une forte inclinaison.
        if (!valid || fabsf(pitch) > TILT_DEG || fabsf(roll) > TILT_DEG) return false;
        switch (mode) {
        case INTERACTIVE_FRONT:
            // Attendre la premiere image du nouveau mode avant de marcher.
            if (vision_ready) follow_line_step();
            return true;
        case INTERACTIVE_LEFT:
            tof_step(now, tof_ok);
            break;
        case INTERACTIVE_BACK: // Suivi de balle reserve pour une prochaine implementation.
        case INTERACTIVE_RIGHT:
            pose(1500, 1500, HEAD_UP_US);
            break;
        }
    } else if (phase == FOLD) {
        if (pose(-1, 2000, HEAD_UP_US)) phase = CENTER;
    } else if (phase == CENTER) {
        if (pose(1500, 2000, HEAD_UP_US)) {
            phase = WAIT_LEVEL;
            level_timing = false;
        }
    } else if (phase == WAIT_LEVEL) {
        if (valid && !gap && fabsf(pitch) < LEVEL_DEG && fabsf(roll) < LEVEL_DEG) {
            if (!level_timing) { level_since = now; level_timing = true; }
            if (now - level_since >= LEVEL_MS) phase = STAND;
        } else level_timing = false;
    } else if (phase == STAND) {
        // La pose a ete validee une fois dans WAIT_LEVEL. Les mouvements du
        // relevement perturbent l'accelerometre : ne pas redeclencher le repli.
        if (pose(1500, 1500, pending == INTERACTIVE_FRONT ? HEAD_DOWN_US : HEAD_UP_US)) {
            mode = pending;
            phase = ACTIVE;
            candidate_timing = false;
            near_left = near_right = near_front = false;
            gesture = GESTURE_READY;
            servo_reset_gait();
            follow_line_reset();
        }
    }
    return phase == ACTIVE && mode == INTERACTIVE_FRONT;
}
