// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>




//bool debug = false;

void init_servo_ctrl();
void demo(int mode );
void move(float D, float theta_t, float theta_r);
void move_2(float D, float theta_t, float theta_r);
void move_step(float D, float theta_t, float theta_r);

void put_in_position();
void heavy_gate();
void heavy_gate_2(float D, float theta_t, float theta_r);

void nod_head();
void raise_head();
void look_down();

void turn_without_moving(int angle_us);
void recenter(int previous_angle_us);
void wake_up();