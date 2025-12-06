#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include "ov7670.h"

int main() {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    setvbuf(stdout, NULL, _IONBF, 0);

    sleep_ms(2000);  // laisser le temps à l'USB de se monter

    // Init caméra (I2C + GPIO + registres capteur)
    ov7670_init();

    static uint16_t frame[OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT];

    while (true) {
        ov7670_capture_frame(frame);
        ov7670_send_frame_usb(frame);
        // eventuellement un petit sleep_ms(10) si tu veux limiter le flux
    }

    return 0;
}
