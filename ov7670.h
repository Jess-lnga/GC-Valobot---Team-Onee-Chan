#ifndef OV7670_H
#define OV7670_H

#include <stdint.h>

// Taille de l'image produite par le module
#define OV7670_IMG_WIDTH   160
#define OV7670_IMG_HEIGHT  120

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise la caméra OV7670 :
 *  - init I2C (i2c0, 100 kHz)
 *  - configure les pins SDA/SCL
 *  - lance le XCLK via PWM
 *  - configure les GPIO data/VSYNC/HREF/PCLK
 *  - configure les registres internes du capteur (QQVGA 160x120 RGB565)
 *
 * ATTENTION : stdio_usb doit déjà être initialisé dans main().
 */
void ov7670_init(void);

/**
 * Capture une frame RGB565 160x120 dans le buffer donné.
 * Le buffer doit faire au moins OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT uint16_t.
 */
void ov7670_capture_frame(uint16_t *buf);

/**
 * Envoie la frame via USB (stdout) avec l’en-tête "OVF0" + métadonnées.
 * Le buffer doit contenir OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT pixels RGB565.
 */
void ov7670_send_frame_usb(uint16_t *buf);

#ifdef __cplusplus
}
#endif

#endif // OV7670_H
