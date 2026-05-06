// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

#include "ov7670.h"
#include "frame_find_line.h"
#include "frame_find_target.h"
#include "labyrinthe.h"
#include "pca9685.h"
#include "line_following.h"
#include "tof.h"
#include "imu.h"

#define LINE_FOLLOWING_MODE         0
#define RISING_SLOPE_MODE           1
#define MIDDLE_SLOPE_MODE           2
#define FALLING_SLOPE_MODE          3
#define LABYRINTHE_TRANSITION_MODE  4
#define LABYRINTHE_MODE             5
#define SHOOTING_TRANSITION_MODE    6
#define SHOOTING_MODE               7
#define GOING_TO_SPYKE_MODE         8
#define FINISHING_MODE              9
#define REST_MODE                   10

#define RISING_SLOPE_ANGLE_THRESHOLD_DEG  14.0f
#define MIDDLE_SLOPE_ANGLE_THRESHOLD_DEG  3.0f

static volatile bool g_line_following_ready = false;
static volatile bool g_tof_ready = false;
static volatile bool g_imu_ready = false;

static bool debug = true;

static int gc_valobot_mode = LINE_FOLLOWING_MODE; 
/* 
0 = line following, 
1 = rising_slope, 
2 = middle_slope, 
3 = falling_slope, 
4 = labyrinthe_transition, 
5 = labyrinthe, 
6 = shooting_transition
7 = shooting,
8 = going_to_spyke,
9 = finishing 
*/  

static bool ready_to_use_cam = false;
static bool line_following_init = true;




#define RISING_SLOPE_COUNT_THRESHOLD 5
int rising_slope_count = 0;


static void core1_entry(void) {
    ///////////// SERVO CORE - INITIALIZATION /////////////
    init_servo_ctrl();
    wake_up();
    initial_raise_head();

    sleep_ms(1000);
    
    look_down(HEAD_LEVEL_2);

    for(int i = 0; i < 40; ++i){
        mes_all_dist();
        imu_capture();
        sleep_ms(50);
    }

    ready_to_use_cam = true;

    ///////////////////////////////////////////////////////
    while(true){

        if(gc_valobot_mode == LINE_FOLLOWING_MODE){
            
            if(line_following_init){
                look_down(HEAD_LEVEL_2);
                line_following_init = false;
                sleep_ms(1000);
            }

            follow_line_step();

            imu_capture();

            float pitch = get_mean_pitch();

            if(pitch > RISING_SLOPE_ANGLE_THRESHOLD_DEG){
                gc_valobot_mode = RISING_SLOPE_MODE;
            }
            sleep_ms(1);
        }

        if(gc_valobot_mode == RISING_SLOPE_MODE){
            heavy_gate();

            imu_capture();
            float pitch = get_mean_pitch();

            if(abs(pitch) < MIDDLE_SLOPE_ANGLE_THRESHOLD_DEG){
                gc_valobot_mode = MIDDLE_SLOPE_MODE;
            }

        }

        if(gc_valobot_mode == MIDDLE_SLOPE_MODE){
            sleep_ms(5000);
    
        }

        if(gc_valobot_mode == FALLING_SLOPE_MODE){
            sleep_ms(5000);
    
        }

        if(gc_valobot_mode == LABYRINTHE_TRANSITION_MODE){
            bool labyrinthe_transition_done = false;
            
            while(!labyrinthe_transition_done){
                labyrinthe_transition_done = follow_line_step();
                
                mes_all_dist();
                sleep_ms(1);
            }

            raise_head();
            //gc_valobot_mode = LABYRINTHE_MODE;
            gc_valobot_mode = REST_MODE;
        }

        if(gc_valobot_mode == LABYRINTHE_MODE){
            bool maze_solved = false;

            while(!maze_solved){
                
                maze_solved = solve_maze();
                mes_all_dist();

                sleep_ms(5);
            }

            gc_valobot_mode = SHOOTING_TRANSITION_MODE; 
        }

        if(gc_valobot_mode == SHOOTING_TRANSITION_MODE){
            bool shooting_transition_done = false;
            
            while(!shooting_transition_done){
                shooting_transition_done = follow_line_step();
                
                mes_all_dist();
                sleep_ms(1);
            }

            gc_valobot_mode = SHOOTING_MODE;
        }

        if(gc_valobot_mode == SHOOTING_MODE){
            sleep_ms(5000);
        }

        if(gc_valobot_mode == GOING_TO_SPYKE_MODE){
            sleep_ms(5000);
        }

        if(gc_valobot_mode == FINISHING_MODE){
            sleep_ms(5000);
        }
    }
}

static void init_all(void) {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(2000);

    ov7670_init();

    //On repasse i2c0 à 400k après init caméra
    i2c_init(i2c0, 400 * 1000);
    g_tof_ready = tof_init_all();
    g_imu_ready = imu_init();


    sleep_ms(100);

    //gc_valobot_mode = LINE_FOLLOWING_MODE;
    //gc_valobot_mode = SHOOTING_MODE;
    //gc_valobot_mode = RISING_SLOPE_MODE;
    //gc_valobot_mode = LABYRINTHE_MODE;
    //gc_valobot_mode = REST_MODE;
    gc_valobot_mode = LABYRINTHE_TRANSITION_MODE;

    multicore_launch_core1(core1_entry);
}

int main() {
    init_all();

    static uint16_t frame[OV7670_IMG_WIDTH * OV7670_IMG_HEIGHT];

    while (true) {
        if(ready_to_use_cam){
            if((gc_valobot_mode == LINE_FOLLOWING_MODE)&&(!line_following_init)){
                ov7670_capture_frame(frame);
                find_line_pos(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT);
                
                if(debug){
                    ov7670_send_frame_usb(frame);  
                }          
            }

            if(gc_valobot_mode == LABYRINTHE_TRANSITION_MODE){
                ov7670_capture_frame(frame);
                find_line_pos_and_detect_t_shape(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT);
                
                if(debug){
                    ov7670_send_frame_usb(frame);  
                }          
            }

            if(gc_valobot_mode == SHOOTING_MODE){
                ov7670_capture_frame(frame);
                find_targets(frame, OV7670_IMG_WIDTH, OV7670_IMG_HEIGHT);
                
                if(debug){
                    ov7670_send_frame_usb(frame);  
                }          
            }
        }
         
        sleep_ms(1);
    }
}



/*

            imu_capture();
            
            printf(
                "\033[HIMU inst | pitch=%7.2f deg | roll=%7.2f deg | addr=0x%02X        \n"
                "IMU mean | pitch=%7.2f deg | roll=%7.2f deg | n=%2d             ",
                get_instant_pitch(),
                get_instant_roll(),
                imu_get_addr(),
                get_mean_pitch(),
                get_mean_roll(),
                IMU_MEAN_WINDOW
            );
            fflush(stdout);

            sleep_ms(50);
*/
/*
        int d_right;
        int d_front;
        int d_left;
        int mean_d_right;
        int mean_d_front;
        int mean_d_left;

        if (!g_tof_ready) {
            printf("\033[HTOF init failed                                        \n");
            fflush(stdout);
            sleep_ms(250);
            continue;
        }

        //mes_dist_right();
        //mes_dist_front();
        //mes_dist_left ();

        mes_all_dist();

        d_right = get_dist_right();
        d_front = get_dist_front();
        d_left  = get_dist_left ();

        mean_d_right = get_dist_mean_right();
        mean_d_front = get_dist_mean_front();
        mean_d_left  = get_dist_mean_left();

        printf(
            "\033[HTOF mean | R=%4d mm | F=%4d mm | L=%4d mm        \n"
            "TOF inst | R=%4d mm | F=%4d mm | L=%4d mm        ",
            mean_d_right,
            mean_d_front,
            mean_d_left,
            d_right,
            d_front,
            d_left
        );
        fflush(stdout);

        sleep_ms(10);
*/
/*
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

#define LED_PIN 2
#define ALERT_LED_PIN 3
#define BUZZER_PIN 4
#define UART_TX_PIN 8
#define UART_RX_PIN 9

#define ADC_GPIO_0 26
#define ADC_GPIO_1 27
#define ADC_GPIO_2 28
#define ADC_GPIO_3 29

#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_2 2
#define ADC_CHANNEL_3 3
#define ADC_INPUT_COUNT 4

#define ADC_MAX_READING 4095.0f
#define ADC_REF_VOLTAGE_MV 3300.0f
#define ADC_SETTLE_TIME_US 50
#define ADC_INTER_SAMPLE_TIME_US 10
#define SAMPLE_PERIOD_MS 50
#define MOVING_AVERAGE_SIZE 16
#define HISTORY_WINDOW_MS 1000
#define HISTORY_SIZE (HISTORY_WINDOW_MS / SAMPLE_PERIOD_MS)
#define DROP_THRESHOLD_MV 10.0f

#define BUZZER_ACTIVE_LEVEL 1
#define BEEP_DURATION_MS 100
#define BEEP_PAUSE_MS 50
#define ALERT_HOLD_MS 5000
#define RESTART_SETTLE_MS 2000
#define ESP_UART_ID uart1
#define ESP_UART_BAUD 115200

typedef struct {
    uint16_t samples[MOVING_AVERAGE_SIZE];
    uint32_t sum;
    uint8_t index;
    uint8_t count;
} moving_average_t;

typedef struct {
    float samples[HISTORY_SIZE];
    uint8_t index;
    uint8_t count;
} voltage_history_t;

typedef struct {
    bool active;
    bool buzzer_on;
    uint8_t transitions_remaining;
    absolute_time_t next_transition;
} buzzer_pattern_t;

static const uint adc_channels[ADC_INPUT_COUNT] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
};

static void buzzer_set(bool enabled) {
    gpio_put(BUZZER_PIN, enabled ? BUZZER_ACTIVE_LEVEL : !BUZZER_ACTIVE_LEVEL);
}

static void init_outputs(void) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    gpio_init(ALERT_LED_PIN);
    gpio_set_dir(ALERT_LED_PIN, GPIO_OUT);
    gpio_put(ALERT_LED_PIN, 0);

    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    buzzer_set(false);
}

static void init_adc_inputs(void) {
    adc_init();
    adc_gpio_init(ADC_GPIO_0);
    adc_gpio_init(ADC_GPIO_1);
    adc_gpio_init(ADC_GPIO_2);
    adc_gpio_init(ADC_GPIO_3);
}

static void init_esp_uart(void) {
    uart_init(ESP_UART_ID, ESP_UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(ESP_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(ESP_UART_ID, false, false);
    uart_set_fifo_enabled(ESP_UART_ID, true);
}

static void send_trigger_to_esp32(void) {
    uart_puts(ESP_UART_ID, "TRIGGER\r\n");
}

static uint16_t read_adc_raw(uint channel) {
    adc_select_input(channel);
    sleep_us(ADC_SETTLE_TIME_US);
    (void)adc_read();
    sleep_us(ADC_INTER_SAMPLE_TIME_US);
    return adc_read();
}

static float raw_to_mv(uint16_t raw_value) {
    return ((float)raw_value * ADC_REF_VOLTAGE_MV) / ADC_MAX_READING;
}

static void moving_average_init(moving_average_t *filter) {
    filter->sum = 0;
    filter->index = 0;
    filter->count = 0;

    for (uint i = 0; i < MOVING_AVERAGE_SIZE; ++i) {
        filter->samples[i] = 0;
    }
}

static uint16_t moving_average_update(moving_average_t *filter, uint16_t sample) {
    filter->sum -= filter->samples[filter->index];
    filter->samples[filter->index] = sample;
    filter->sum += sample;
    filter->index = (filter->index + 1) % MOVING_AVERAGE_SIZE;

    if (filter->count < MOVING_AVERAGE_SIZE) {
        filter->count++;
    }

    return (uint16_t)(filter->sum / filter->count);
}

static void voltage_history_init(voltage_history_t *history) {
    history->index = 0;
    history->count = 0;

    for (uint i = 0; i < HISTORY_SIZE; ++i) {
        history->samples[i] = 0.0f;
    }
}

static void voltage_history_push(voltage_history_t *history, float sample_mv) {
    history->samples[history->index] = sample_mv;
    history->index = (history->index + 1) % HISTORY_SIZE;

    if (history->count < HISTORY_SIZE) {
        history->count++;
    }
}

static bool voltage_history_is_full(const voltage_history_t *history) {
    return history->count >= HISTORY_SIZE;
}

static float voltage_history_get_oldest(const voltage_history_t *history) {
    return history->samples[history->index];
}

static void reset_detection_state(
    moving_average_t filters[ADC_INPUT_COUNT],
    voltage_history_t histories[ADC_INPUT_COUNT]
) {
    for (uint i = 0; i < ADC_INPUT_COUNT; ++i) {
        moving_average_init(&filters[i]);
        voltage_history_init(&histories[i]);
    }
}

static void start_buzzer_pattern(buzzer_pattern_t *pattern) {
    pattern->active = true;
    pattern->buzzer_on = true;
    pattern->transitions_remaining = 3;
    pattern->next_transition = delayed_by_ms(get_absolute_time(), BEEP_DURATION_MS);
    buzzer_set(true);
}

static void update_buzzer_pattern(buzzer_pattern_t *pattern) {
    if (!pattern->active) {
        return;
    }

    if (absolute_time_diff_us(get_absolute_time(), pattern->next_transition) > 0) {
        return;
    }

    pattern->buzzer_on = !pattern->buzzer_on;
    buzzer_set(pattern->buzzer_on);

    if (pattern->transitions_remaining > 0) {
        pattern->transitions_remaining--;
    }

    if (pattern->transitions_remaining == 0) {
        pattern->active = false;
        pattern->buzzer_on = false;
        buzzer_set(false);
        return;
    }

    pattern->next_transition = delayed_by_ms(get_absolute_time(), BEEP_PAUSE_MS);
}

int main(void) {
    uint16_t raw_samples[ADC_INPUT_COUNT] = {0};
    uint16_t filtered_samples[ADC_INPUT_COUNT] = {0};
    float voltages_mv[ADC_INPUT_COUNT] = {0.0f};
    moving_average_t filters[ADC_INPUT_COUNT];
    voltage_history_t histories[ADC_INPUT_COUNT];
    buzzer_pattern_t buzzer_pattern = {0};
    bool system_ready = false;
    bool alert_hold_active = false;
    absolute_time_t alert_hold_end_time = nil_time;
    bool restart_settle_active = false;
    absolute_time_t restart_settle_end_time = nil_time;

    init_outputs();
    stdio_init_all();
    sleep_ms(2000);
    init_adc_inputs();
    init_esp_uart();

    reset_detection_state(filters, histories);

    while (true) {
        bool voltage_drop_detected = false;

        update_buzzer_pattern(&buzzer_pattern);

        if (
            alert_hold_active &&
            absolute_time_diff_us(get_absolute_time(), alert_hold_end_time) <= 0
        ) {
            alert_hold_active = false;
            gpio_put(ALERT_LED_PIN, 0);
            reset_detection_state(filters, histories);
            system_ready = false;
            restart_settle_active = true;
            restart_settle_end_time = delayed_by_ms(get_absolute_time(), RESTART_SETTLE_MS);
        }

        if (
            restart_settle_active &&
            absolute_time_diff_us(get_absolute_time(), restart_settle_end_time) <= 0
        ) {
            restart_settle_active = false;
        }

        for (uint i = 0; i < ADC_INPUT_COUNT; ++i) {
            float reference_mv = 0.0f;

            raw_samples[i] = read_adc_raw(adc_channels[i]);
            filtered_samples[i] = moving_average_update(&filters[i], raw_samples[i]);
            voltages_mv[i] = raw_to_mv(filtered_samples[i]);

            if (
                !alert_hold_active &&
                !restart_settle_active &&
                system_ready &&
                voltage_history_is_full(&histories[i])
            ) {
                reference_mv = voltage_history_get_oldest(&histories[i]);

                if ((reference_mv - voltages_mv[i]) >= DROP_THRESHOLD_MV) {
                    voltage_drop_detected = true;
                }
            }

            voltage_history_push(&histories[i], voltages_mv[i]);
        }

        if (
            voltage_drop_detected &&
            !alert_hold_active &&
            !restart_settle_active
        ) {
            start_buzzer_pattern(&buzzer_pattern);
            send_trigger_to_esp32();
            gpio_put(ALERT_LED_PIN, 1);
            alert_hold_active = true;
            alert_hold_end_time = delayed_by_ms(get_absolute_time(), ALERT_HOLD_MS);
        }

        if (!alert_hold_active && !restart_settle_active) {
            system_ready = true;
        }

        printf(
            "\rA0:%4u/%4u %.1fmV | A1:%4u/%4u %.1fmV | A2:%4u/%4u %.1fmV | A3:%4u/%4u %.1fmV | B:%s | L:%s | S:%s    ",
            raw_samples[0], filtered_samples[0], voltages_mv[0],
            raw_samples[1], filtered_samples[1], voltages_mv[1],
            raw_samples[2], filtered_samples[2], voltages_mv[2],
            raw_samples[3], filtered_samples[3], voltages_mv[3],
            buzzer_pattern.active ? "ON " : "OFF",
            alert_hold_active ? "ON " : "OFF",
            restart_settle_active ? "ON " : "OFF"
        );
        fflush(stdout);

        sleep_ms(SAMPLE_PERIOD_MS);
    }

    return 0;
}

*/
