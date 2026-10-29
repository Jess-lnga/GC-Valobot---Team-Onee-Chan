// Test natif de la vraie FSM, avec capteurs et servos simules.
// cl /nologo /W4 /std:c11 /Itests/stubs /Febuild/test_interactive.exe
//    /Fobuild/test_interactive.obj tests/test_interactive.c
#include <assert.h>
#include <stdio.h>
#include "../interactive.c"

static float test_pitch, test_roll;
static bool fresh = true, pose_reached = true, head_reached = true;
static int dl = 1000, dr = 1000, df = 1000, line_calls, last_head;
static int last_c[4], last_t[4];
static uint64_t clock_ms;

bool imu_capture(void) { return fresh; }
float get_instant_pitch(void) { return test_pitch; }
float get_instant_roll(void) { return test_roll; }
void mes_all_dist(void) {}
int get_dist_left(void) { return dl; }
int get_dist_right(void) { return dr; }
int get_dist_front(void) { return df; }
bool follow_line_step(void) { ++line_calls; return true; }
void follow_line_reset(void) {}
void servo_reset_gait(void) {}
bool servo_pose_step(const int c[4], const int t[4], int h) {
    for (int i = 0; i < 4; ++i) { last_c[i] = c[i]; last_t[i] = t[i]; }
    if (h >= 0) last_head = h;
    if (c[0] < 0 && t[0] < 0) return head_reached;
    return pose_reached;
}
bool servo_pose_step_speed(const int c[4], const int t[4], int h, int step_us) {
    assert(step_us == ((gesture == GESTURE_TWIST || gesture == GESTURE_RETURN)
                       ? TOF_TWIST_STEP_US : TOF_LEG_STEP_US));
    return servo_pose_step(c, t, h);
}

static bool tick(void) {
    clock_ms += 20;
    return interactive_step(clock_ms, true, true, true);
}
static void reset(void) {
    interactive_init();
    clock_ms = 0;
    test_pitch = test_roll = 0;
    fresh = pose_reached = true;
    head_reached = true;
    dl = dr = df = 1000;
    line_calls = 0;
}
static void hold(int ms) { for (int i = 0; i < ms / 20; ++i) tick(); }
static void select_mode(float p, float r, interactive_mode_t expected) {
    test_pitch = p; test_roll = r;
    tick(); hold(2980);
    assert(phase == ACTIVE);
    tick();
    assert(phase == FOLD && pending == expected);
    pose_reached = false;
    hold(200);
    assert(phase == FOLD);
    pose_reached = true;
    tick(); assert(phase == CENTER);
    tick(); assert(phase == WAIT_LEVEL);
    hold(1000); assert(phase == WAIT_LEVEL); // Toujours incline.
    test_pitch = test_roll = 0;
    tick(); hold(480); assert(phase == WAIT_LEVEL);
    tick(); assert(phase == STAND);
    tick(); assert(phase == ACTIVE && mode == expected);
}

int main(void) {
    reset(); select_mode(-31, 0, INTERACTIVE_FRONT);
    int before = line_calls;
    clock_ms += 20;
    assert(interactive_step(clock_ms, true, true, false));
    assert(line_calls == before); // Pas de marche avant la premiere image.
    assert(tick()); assert(line_calls == before + 1);
    // Transition depuis un mode actif, et Back reste sans suivi de balle.
    select_mode(31, 0, INTERACTIVE_BACK);
    before = line_calls; hold(1000); assert(line_calls == before);
    select_mode(0, -31, INTERACTIVE_LEFT);
    select_mode(0, 31, INTERACTIVE_RIGHT);

    reset(); test_pitch = -31; hold(2000);
    test_pitch = 0; tick(); test_pitch = -31;
    hold(1500); assert(phase == ACTIVE); // Pas de cumul de gestes interrompus.
    test_roll = 31; hold(3500); assert(phase == ACTIVE); // Diagonale ambigue.
    test_roll = 0; hold(2000); fresh = false; tick(); fresh = true;
    hold(1500); assert(phase == ACTIVE); // Une erreur casse le maintien.
    clock_ms += 500; tick(); hold(2000); assert(phase == ACTIVE);
    test_pitch = -30; hold(3500); assert(phase == ACTIVE); // Seuil strict.

    reset(); mode = INTERACTIVE_LEFT;
    dl = 100; hold(200); assert(gesture == GESTURE_TWIST);
    assert(last_c[0] == LEFT_TURN_US && last_t[0] == 1500);
    dl = 175; hold(200); assert(gesture == GESTURE_TWIST); // Hysteresis.
    dl = 250; hold(200); assert(gesture == GESTURE_READY);
    assert(last_c[0] == 1500);
    dl = 100; pose_reached = false; hold(200);
    dl = 1000; hold(200);
    assert(gesture == GESTURE_TWIST && side_released);
    pose_reached = true; hold(200);
    assert(gesture == GESTURE_READY); // Le geste bref finit meme si la main est retiree.
    dl = dr = 100; hold(2000); assert(gesture == GESTURE_READY);
    dl = 1000; hold(2000); assert(gesture == GESTURE_WAIT_RELEASE);
    assert(last_c[0] == 1500 && last_t[0] == 1500);
    hold(2000); assert(gesture == GESTURE_WAIT_RELEASE); // Une rotation par geste.
    dr = 1000; hold(200); assert(gesture == GESTURE_READY);
    df = 100; hold(200); assert(last_head == HEAD_DOWN_US);
    df = -1; hold(200); assert(last_head == HEAD_UP_US);
    head_reached = false; df = 100; dl = 100;
    hold(2000);
    assert(last_head == SERVO_HEAD_LINE_US);
    assert(gesture == GESTURE_WAIT_RELEASE); // La rampe de tete ne bloque pas les pattes.
    assert(HEAD_UP_US > 1100 && HEAD_UP_US < HEAD_DOWN_US);

    reset(); phase = WAIT_LEVEL; pending = INTERACTIVE_RIGHT;
    hold(400); fresh = false; tick(); fresh = true;
    hold(400); assert(phase == WAIT_LEVEL);
    hold(200); assert(phase == ACTIVE);
    phase = STAND; pose_reached = false;
    test_roll = 6; hold(200); assert(phase == STAND);
    test_pitch = 15; hold(200); assert(phase == STAND);
    fresh = false; hold(200); assert(phase == STAND);
    pose_reached = true; tick(); assert(phase == ACTIVE);
    puts("FSM interactive : tous les scenarios passent.");
    return 0;
}
