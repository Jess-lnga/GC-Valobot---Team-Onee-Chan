// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA

#ifndef OV7670_PIO_DMA_H
#define OV7670_PIO_DMA_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"   // définit uint

bool ov7670_pio_dma_init(uint data_base_pin,
                         uint pclk_pin,
                         uint vsync_pin,
                         uint href_pin,
                         int width,
                         int height);

bool ov7670_pio_dma_capture(uint16_t *buf);

#endif // OV7670_PIO_DMA_H
