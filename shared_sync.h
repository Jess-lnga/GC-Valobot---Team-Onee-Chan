#ifndef SHARED_SYNC_H
#define SHARED_SYNC_H

#include <stdbool.h>
#include <stdint.h>

// Flag mis à true pendant la capture (core0), lu par core1 pour éviter l'I2C
extern volatile bool g_capture_active;

#endif
