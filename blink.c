// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA [Alone :( ]

#include <stdio.h> 
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#define BUFF_SIZE 16
int main() {
    stdio_init_all();

    static char buff[BUFF_SIZE];

    // (Optionnel) attendre qu'un terminal ouvre le port série USB
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }

    puts("USB ready. Type something...\r\n");
    
    int buff_index = 0;

    while (true) {
        int ch = getchar_timeout_us(0);              // -1 ( = PICO_ERROR_TIMEOUT) si rien

        while (ch != PICO_ERROR_TIMEOUT){
            buff[buff_index] = (char)ch;
            ++buff_index;

            if(buff_index >= BUFF_SIZE - 1){
                puts("Try to type something shorter");

                while(ch != PICO_ERROR_TIMEOUT){
                    ch = getchar_timeout_us(0);
                }
            }else{
                ch = getchar_timeout_us(0);
            }       
        }

        if(buff_index > 0){
            buff[buff_index] = '\0';

            puts("Echo from RP2040: \r\n");
            printf("%s\r\n", buff);

            buff_index = 0;
        }
        
        sleep_ms(1);
        tight_loop_contents();                       // idle hint
    }
}

