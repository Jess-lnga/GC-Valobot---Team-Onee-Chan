#ifndef OV7670_H
#define OV7670_H

#include <stdint.h>

#define OV7670_IMG_WIDTH   160
#define OV7670_IMG_HEIGHT  120

#ifdef __cplusplus
extern "C" {
#endif

void ov7670_init(void);

// Capture une frame RGB565 160x120 dans buf (PIO + DMA).
// buf doit faire OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT uint16_t.
void ov7670_capture_frame(uint16_t *buf);

// Envoi binaire sur stdout (USB CDC) : "OVF0" + header + pixels.
void ov7670_send_frame_usb(uint16_t *buf);

#ifdef __cplusplus
}
#endif

#endif // OV7670_H
