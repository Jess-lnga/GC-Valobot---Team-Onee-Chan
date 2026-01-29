#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

#include "ov7670.h"
#include "frame_analysis.h"
#include "pca9685.h"

// ------------------------------------------------------------
// Core1 : servos (PCA9685)
// ------------------------------------------------------------
static void core1_entry(void) {
    // Init PCA9685 (I2C 400kHz)
    init_servo_ctrl();

    int mode = 6;

    while (true) {
        // Optionnel : recevoir un mode/commande depuis core0
        if (multicore_fifo_rvalid()) {
            mode = (int)multicore_fifo_pop_blocking();
        }

        // Mouvement (bloquant, mais sur core1 seulement)
        demo(mode);
    }
}

// ------------------------------------------------------------
// Init global (core0)
// ------------------------------------------------------------
static void init_all(void) {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(2000);  // temps pour que l'USB monte

    // 1) Init caméra (I2C utilisé ici)
    ov7670_init();

    // 2) IMPORTANT : après config caméra, on repasse I2C à 400kHz
    // pour le PCA9685 (la capture n'utilise pas l'I2C)
    i2c_init(i2c0, 400 * 1000);

    // 3) Lance core1 (servos)
    multicore_launch_core1(core1_entry);
}

int main() {
    init_all();

    static uint16_t frame[OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT];

    int mode = 0;

    while (true) {
        // --- Tâche core0 : capture + analyse + envoi USB ---
        ov7670_capture_frame(frame);
        /*
        line_detection_t det;
        frame_analyze_line_rgb565(
            frame,
            OV7670_IMG_WIDTH,
            OV7670_IMG_HEIGHT,
            &det
        );
        */

        ov7670_send_frame_usb(frame);

        // --- Optionnel : envoyer un "mode" au core1 ---
        // Exemple bête : change de mode si une ligne est trouvée
        // (Tu remplaceras par ton vrai contrôle plus tard)
        /*
        if (det.found) {
            // exemple : mode dépend de la position du centre
            // gauche / centre / droite
            if (det.center_x < (OV7670_IMG_WIDTH / 3)) mode = 1;
            else if (det.center_x > (2 * OV7670_IMG_WIDTH / 3)) mode = 2;
            else mode = 0;

            // envoi non-bloquant : si FIFO pleine, on saute (évite de bloquer la capture)
            if (multicore_fifo_wready()) {
                multicore_fifo_push_blocking((uint32_t)mode);
            }
        }
        */
    }

    return 0;
}
