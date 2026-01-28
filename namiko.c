/*
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"


#define BUZZER_GPIO 15
#define LED_GPIO 25

// ADC: GP26 = ADC0
#define ADC_GPIO 26
#define ADC_CH   0

// --- Réglages ---
// Seuil minimum de baisse pour déclencher (en Volts).
// Avec 3.3V/4095 ≈ 0.000806 V par LSB, 10 mV ≈ 12-13 LSB.
#define DROP_THRESHOLD_V 0.002f   // 10 mV

// Bip quand baisse détectée
#define BEEP_MS 150

// Période de mesure
#define SAMPLE_PERIOD_MS 20

static inline void beep(uint32_t ms) {
    gpio_put(BUZZER_GPIO, 1);
    gpio_put(LED_GPIO, 1);
    sleep_ms(ms);
    gpio_put(BUZZER_GPIO, 0);
    gpio_put(LED_GPIO, 0);
}

int main(void) {
    stdio_init_all();

    // GPIO buzzer + LED
    gpio_init(BUZZER_GPIO);
    gpio_set_dir(BUZZER_GPIO, GPIO_OUT);
    gpio_put(BUZZER_GPIO, 0);

    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_put(LED_GPIO, 0);

    // ADC init
    adc_init();
    adc_gpio_init(ADC_GPIO);
    adc_select_input(ADC_CH);

    // Petit délai pour que l'USB-serial soit prêt (optionnel)
    sleep_ms(500);

    const float vref = 3.3f;

    // Lecture initiale
    uint16_t raw_prev = adc_read();
    float v_prev = (raw_prev * vref) / 4095.0f;

    while (true) {
        uint16_t raw = adc_read();
        float v = (raw * vref) / 4095.0f;

        // Debug (optionnel)
        // printf("V=%.3f V (prev %.3f)\n", v, v_prev);

        // Détection baisse
        //float dv = v_prev - v;  // positif si ça baisse
        float dv = v - v_prev;  // positif si ça baisse
        if (dv >= DROP_THRESHOLD_V) {
            beep(BEEP_MS);
        }

        v_prev = v;
        sleep_ms(SAMPLE_PERIOD_MS);
    }
}
*/
#include "pca9685.h"

#define FORWARD 6
#define TURN_AROUND 3
#define UP_N_DOWN 0

int main(){
    init_servo_ctrl();

    while(true){
        /*
        for(int i = 0; i < 10; ++i){
            demo(6);
        }
        sleep_ms(1000);

        demo(0);
        sleep_ms(1000);
        

        for(int i = 0; i < 5; ++i){
            demo(3);
        }
        sleep_ms(1000);

        demo(0);
        sleep_ms(1000);
        */

        //demo(TURN_AROUND);
        demo(FORWARD);
    }
    
    return 0;
}
