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
    int last_mm;
    bool started;
} tof_t;

bool tof_init_all(void);

void mes_dist_right(void);
void mes_dist_front(void);
void mes_dist_left(void);

void mes_all_dist(void);

int get_dist_right(void);
int get_dist_front(void);
int get_dist_left(void);

int get_dist_mean_right(void);
int get_dist_mean_front(void);
int get_dist_mean_left(void);



void tof_stop_all(void);

void pca_is_active(void);
void pca_is_not_active(void);


bool tof_read_model_id(tof_t *t, uint8_t *id);
bool tof_read_irq_status(tof_t *t, uint8_t *st);

#endif
