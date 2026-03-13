#pragma once

#include "pca9685.h"
#include "frame_analysis.h"

// Anciennes fonctions éventuellement encore utilisées ailleurs
void follow_line_testing(void);
void follow_line(void);

// Suivi de ligne
void follow_line_reset(void);
void follow_line_step(void);

// Alignement sur cible bleue
void target_align_reset(void);
void target_align_step(void);