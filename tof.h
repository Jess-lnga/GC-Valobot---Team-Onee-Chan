#ifndef TOF_H
#define TOF_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

#define TOF_DEFAULT_ADDR 0x29

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint8_t xshut_pin;
    uint8_t stop_variable;
    uint32_t io_timeout_us;
    bool started;
} tof_t;

bool tof_init_all(void);

int get_dist_1(void);
int get_dist_2(void);
int get_dist_3(void);

void tof_stop_all(void);

bool tof_read_model_id(tof_t *t, uint8_t *id);
bool tof_read_irq_status(tof_t *t, uint8_t *st);

#endif
