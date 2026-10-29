#ifndef INTERACTIVE_H
#define INTERACTIVE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    INTERACTIVE_RIGHT, INTERACTIVE_FRONT, INTERACTIVE_BACK, INTERACTIVE_LEFT
} interactive_mode_t;

void interactive_init(void);
// Appeler depuis le coeur servo uniquement. Retourne true si le suivi de ligne est actif.
bool interactive_step(uint64_t now_ms, bool imu_ok, bool tof_ok, bool vision_ready);

#endif
