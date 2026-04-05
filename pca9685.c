// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA

#include "pca9685.h"

#define I2C_PORT      i2c0
#define I2C_SDA_PIN   4      // GP4
#define I2C_SCL_PIN   5      // GP5
#define PCA_ADDR      0x40   // adresse par défaut

// Registres PCA9685
#define MODE1         0x00
#define MODE2         0x01
#define PRE_SCALE     0xFE
#define LED0_ON_L     0x06   // LEDn_ON_L = 0x06 + 4*n

#define COXA_FL 11
#define COXA_FR 4
#define COXA_BL 9
#define COXA_BR 7
 
#define TROC_FL 10
#define TROC_FR 6
#define TROC_BL 8
#define TROC_BR 5

#define INVALID_CHANNEL -1

#define LEG_L (float)4.9
#define LEG_R (float)6.0 

static int servo_offset[16] = {0};

// ------------------------------------------------ //
//bool debug = false;
bool initialize = true;
int count = 0;


const float us_par_tick = (1000000.0f / 50.0f) / 4096.0f;
// ------------------------------------------------ //

// Convertit des microsecondes en ticks (sur 12 bits)
static inline uint16_t to_ticks_us(int us, float us_per_tick) {
    return (uint16_t)( (us / us_per_tick) + 0.5f ); // arrondi au plus proche
}

static void pca_write8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    (void)i2c_write_blocking(I2C_PORT, PCA_ADDR, buf, 2, false);
}
/// ////////////////////////////////////////////////////////////////////////// ///
static void pca_write_pwm_offset(uint8_t channel, uint16_t on, uint16_t off, int offset) {
    int corrected_off = (int)off + offset;

    int min_ticks = to_ticks_us(1000, us_par_tick);
    int max_ticks = to_ticks_us(2000, us_par_tick);

    if (corrected_off < min_ticks) {
        corrected_off = min_ticks;
    }
    if (corrected_off > max_ticks) {
        corrected_off = max_ticks;
    }

    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        reg,
        on & 0xFF,
        on >> 8,
        corrected_off & 0xFF,
        corrected_off >> 8
    };

    (void)i2c_write_blocking(I2C_PORT, PCA_ADDR, buf, 5, false);
}


static void pca_write_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    //uint8_t reg = LED0_ON_L + 4 * channel;
    //uint8_t buf[5] = {reg, on & 0xFF, on >> 8, off & 0xFF, off >> 8};
    //(void)i2c_write_blocking(I2C_PORT, PCA_ADDR, buf, 5, false);
    
    if (channel >= 16) {
        return;
    }

    
    pca_write_pwm_offset(channel, on, off, to_ticks_us(servo_offset[channel], us_par_tick));
}

/// ////////////////////////////////////////////////////////////////////////// ///

static uint8_t pca_read8(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(I2C_PORT, PCA_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, PCA_ADDR, &val, 1, false);
    return val;
}

static void pca_set_freq(float freq_hz) {
    // prescale = round(25MHz / (4096 * freq)) - 1
    float prescale_f = 25000000.0f / (4096.0f * freq_hz) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

    // Lire l'état actuel
    uint8_t oldmode = pca_read8(MODE1);

    // Mettre en SLEEP pour pouvoir écrire PRE_SCALE
    uint8_t sleep = (oldmode & ~0x80) | 0x10; // clear RESTART (bit7), set SLEEP (bit4)
    pca_write8(MODE1, sleep);
    sleep_ms(1);

    // Programmer le prescaler
    pca_write8(PRE_SCALE, prescale);

    // Réveiller (SLEEP=0) en restaurant oldmode sans le bit SLEEP
    uint8_t wake = oldmode & ~0x10;
    pca_write8(MODE1, wake);
    sleep_ms(5);

    // Activer AI (auto-increment) + ALLCALL si tu veux + lancer un RESTART
    uint8_t run = (wake | 0x20 | 0x01); // AI=0x20, ALLCALL=0x01
    pca_write8(MODE1, run | 0x80);      // écrire RESTART=1
    sleep_ms(1);

    // Sorties en totem-pole
    pca_write8(MODE2, 0x04);

    // DEBUG: relire et afficher
    uint8_t m1 = pca_read8(MODE1);
    uint8_t m2 = pca_read8(MODE2);
    uint8_t ps = pca_read8(PRE_SCALE);
    //printf("After init: MODE1=0x%02X MODE2=0x%02X PRESCALE=0x%02X\r\n", m1, m2, ps);
}

static void i2c_scan(void) {
    //printf("I2C scan:\r\n");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        uint8_t dummy = 0;
        int r = i2c_write_blocking(I2C_PORT, addr, &dummy, 1, false);
        if (r >= 0) /*printf(" - Found device at 0x%02X\r\n", addr)*/;
    }
}

void init_servo_ctrl(){
    //stdio_init_all();

    servo_offset[COXA_BL] = 100;
    servo_offset[COXA_BR] = 0;
    servo_offset[COXA_FL] = 10;
    servo_offset[COXA_FR] = 0;
    
    servo_offset[TROC_BL] = 0;
    servo_offset[TROC_BR] = 100;
    servo_offset[TROC_FL] = 0;
    servo_offset[TROC_FR] = -50;

    // Init I2C à 400 kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    //i2c_scan();

    // Config PCA9685 à 50 Hz (servos)
    pca_set_freq(50.0f);

}

void move_2(float D, float theta_t, float theta_r){
    //////////////////////////////////////////////////////////////////
    int ticks_min = 1000;
    int ticks_max = 3000 - ticks_min;
    int ticks_middle = 1500;

    float angle_deg_max = 45; // Previously 34

    float angle_rad_max = (angle_deg_max*M_PI)/180;
    float slope = (ticks_max - ticks_middle)/angle_rad_max;

    float d_l = D*sin(M_PI/4.0 - theta_t);
    float d_r = D*cos(M_PI/4.0 - theta_t);

    //////////////////////////////////////////////////////////////////

   //------- Compute mouvement for right diagonal -------//
    float theta_1r = asin((2*d_r + LEG_L*sin(theta_r))/(2*LEG_R)) + theta_r;
    float theta_2r = asin((2*d_r - LEG_L*sin(theta_r))/(2*LEG_R)) - theta_r;
    
    // --> Conversion for digitalisation
    float digit_theta_1r = 1500 - slope * theta_1r; // For COXA_FL
    float digit_theta_2r = 1500 + slope * theta_2r; // For COXA_BR

    float _digit_theta_1r = 1500 - slope * (-1 * theta_1r); // For COXA_FL   --> Because we have a symetrical mvmnt
    float _digit_theta_2r = 1500 + slope * (-1 * theta_2r); // For COXA_BR

    if(digit_theta_1r > ticks_max){digit_theta_1r = ticks_max;}
    if(digit_theta_1r < ticks_min){digit_theta_1r = ticks_min;}
    if(digit_theta_2r > ticks_max){digit_theta_2r = ticks_max;}
    if(digit_theta_2r < ticks_min){digit_theta_2r = ticks_min;}

    if(_digit_theta_1r > ticks_max){_digit_theta_1r = ticks_max;}
    if(_digit_theta_1r < ticks_min){_digit_theta_1r = ticks_min;}
    if(_digit_theta_2r > ticks_max){_digit_theta_2r = ticks_max;}
    if(_digit_theta_2r < ticks_min){_digit_theta_2r = ticks_min;}


    //------- Compute mouvement for left diagonal -------//
    float theta_1l = asin((2*d_l + LEG_L*sin(theta_r))/(2*LEG_R)) + theta_r;
    float theta_2l = asin((2*d_l - LEG_L*sin(theta_r))/(2*LEG_R)) - theta_r;
    
    // --> Conversion for digitalisation
    float digit_theta_1l = 1500 - slope * theta_1l; // For COXA_BL
    float digit_theta_2l = 1500 + slope * theta_2l; // For COXA_FR

    float _digit_theta_1l = 1500 - slope * (-1 * theta_1l); // For COXA_BL
    float _digit_theta_2l = 1500 + slope * (-1 * theta_2l); // For COXA_FR

    if(digit_theta_1l > ticks_max){digit_theta_1l = ticks_max;}
    if(digit_theta_1l < ticks_min){digit_theta_1l = ticks_min;}
    if(digit_theta_2l > ticks_max){digit_theta_2l = ticks_max;}
    if(digit_theta_2l < ticks_min){digit_theta_2l = ticks_min;}

    if(_digit_theta_1l > ticks_max){_digit_theta_1l = ticks_max;}
    if(_digit_theta_1l < ticks_min){_digit_theta_1l = ticks_min;}
    if(_digit_theta_2l > ticks_max){_digit_theta_2l = ticks_max;}
    if(_digit_theta_2l < ticks_min){_digit_theta_2l = ticks_min;}    

    //////////////////////////////////////////////////////////////////

    //------- Mouvement parameters -------//
    int air_step   = 22;   // Good values: air --> 25; floor --> 15
    int floor_step = 35;   // Good values: air --> 22; floor --> 18

    int step = floor_step;

    int min = 1200;
    int max = 2000;
    int middle = 1500;

    //////////////////////////////////////////////////////////////////

    //---------------------------- Mouvement for right side ----------------------------//
    float troc_max = 2000;
    float troc_min = 1500;

    // Transition for mouvement:
    float start_us1r = _digit_theta_1r;
    float stop_us1r  = digit_theta_1r;
    float slope_us1r = (stop_us1r - start_us1r);
    
    float start_us2r = _digit_theta_2r; 
    float stop_us2r  = digit_theta_2r;
    float slope_us2r = (stop_us2r - start_us2r);

    if(initialize){ 
        start_us1r = 1500; 
        start_us2r = 1500; 

        slope_us1r = (stop_us1r - start_us1r);
        slope_us2r = (stop_us2r - start_us2r);

        initialize = false;
    }

    for (int i = 0; i <= floor_step;  ++i){ 
    
        int us1r = start_us1r + i*slope_us1r/(floor_step*1.0);
        int us2r = start_us2r + i*slope_us2r/(floor_step*1.0);

        int us_troc;

        if(i < floor_step/2){
            us_troc = troc_min + i*2*(troc_max - troc_min)/(floor_step*1.0);
        }else{
            us_troc = troc_max - (i - floor_step/2)*2*(troc_max - troc_min)/(floor_step*1.0);
        }
        

        pca_write_pwm(COXA_FL, 0, to_ticks_us(us1r, us_par_tick));
        pca_write_pwm(COXA_BR, 0, to_ticks_us(us2r, us_par_tick)); 
        
        pca_write_pwm(TROC_FL, 0, to_ticks_us(us_troc, us_par_tick));
        pca_write_pwm(TROC_BR, 0, to_ticks_us(us_troc, us_par_tick));

        sleep_ms(10);
    }

    //---------------------------- Mouvement for both sides ----------------------------//

    start_us1r = digit_theta_1r;
    stop_us1r  = 1500;
    slope_us1r = (stop_us1r - start_us1r);
    
    start_us2r = digit_theta_2r;
    stop_us2r  = 1500;
    slope_us2r = (stop_us2r - start_us2r);

    float start_us1l = 1500;
    float stop_us1l  = _digit_theta_1l;
    float slope_us1l = (stop_us1l - start_us1l);
    
    float start_us2l = 1500;
    float stop_us2l  = _digit_theta_2l;
    float slope_us2l = (stop_us2l - start_us2l);

    for (int i = 0; i <= floor_step;  ++i){ 

        int us1r = start_us1r + i*slope_us1r/(floor_step*1.0);
        int us2r = start_us2r + i*slope_us2r/(floor_step*1.0);

        int us1l = start_us1l + i*slope_us1l/(floor_step*1.0);
        int us2l = start_us2l + i*slope_us2l/(floor_step*1.0);

        pca_write_pwm(COXA_FL, 0, to_ticks_us(us1r, us_par_tick));
        pca_write_pwm(COXA_BR, 0, to_ticks_us(us2r, us_par_tick));

        pca_write_pwm(COXA_BL, 0, to_ticks_us(us1l, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us2l, us_par_tick));

        sleep_ms(10);
    }


    //---------------------------- Mouvement for Left side -----------------------------//
    //for (int us = middle; us <= max; us += air_step) {               //Rise the legs
    //    pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
    //    pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick)); 
    //    
    //    sleep_ms(10);
    //}

    // Transition for mouvement:

    start_us1l = _digit_theta_1l;
    stop_us1l  = digit_theta_1l;
    slope_us1l = (stop_us1l - start_us1l);
    
    start_us2l = _digit_theta_2l;
    stop_us2l  = digit_theta_2l;
    slope_us2l = (stop_us2l - start_us2l);

    for (int i = 0; i <= floor_step;  ++i){

        int us1l = start_us1l + i*slope_us1l/(floor_step*1.0);
        int us2l = start_us2l + i*slope_us2l/(floor_step*1.0);

        int us_troc;

        if(i < floor_step/2){
            us_troc = troc_min + i*2*(troc_max - troc_min)/(floor_step*1.0);
        }else{
            us_troc = troc_max - (i - floor_step/2)*2*(troc_max - troc_min)/(floor_step*1.0);
        }

        pca_write_pwm(COXA_BL, 0, to_ticks_us(us1l, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us2l, us_par_tick));

        pca_write_pwm(TROC_BL, 0, to_ticks_us(us_troc, us_par_tick));
        pca_write_pwm(TROC_FR, 0, to_ticks_us(us_troc, us_par_tick));

        sleep_ms(10);
    }

    //for (int us = max; us >= middle; us -= air_step) {              //Get the legs down
    //    pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
    //    pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));  

    //    sleep_ms(10);
    //}

    //---------------------------- Mouvement for both sides ----------------------------//
    start_us1r = 1500;
    stop_us1r  = _digit_theta_1r;
    slope_us1r = (stop_us1r - start_us1r);
    
    start_us2r = 1500;
    stop_us2r  = _digit_theta_2r;
    slope_us2r = (stop_us2r - start_us2r);
    
    ////

    start_us1l = digit_theta_1l;
    stop_us1l  = 1500;
    slope_us1l = (stop_us1l - start_us1l);
    
    start_us2l = digit_theta_2l;
    stop_us2l  = 1500;
    slope_us2l = (stop_us2l - start_us2l);

    for (int i = 0; i <= floor_step;  ++i){ 

        int us1r = start_us1r + i*slope_us1r/(floor_step*1.0);
        int us2r = start_us2r + i*slope_us2r/(floor_step*1.0);

        int us1l = start_us1l + i*slope_us1l/(floor_step*1.0);
        int us2l = start_us2l + i*slope_us2l/(floor_step*1.0);

        pca_write_pwm(COXA_FL, 0, to_ticks_us(us1r, us_par_tick));
        pca_write_pwm(COXA_BR, 0, to_ticks_us(us2r, us_par_tick));

        pca_write_pwm(COXA_BL, 0, to_ticks_us(us1l, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us2l, us_par_tick));

        sleep_ms(10);

    }

}

void move(float D, float theta_t, float theta_r){
    int ticks_min = 1000;
    int ticks_max = 3000 - ticks_min;
    int ticks_middle = 1500;

    float angle_deg_max = 45; // Previously 34

    float angle_rad_max = (angle_deg_max*M_PI)/180;
    float slope = (ticks_max - ticks_middle)/angle_rad_max;

    float d_l = D*sin(M_PI/4.0 - theta_t);
    float d_r = D*cos(M_PI/4.0 - theta_t);

   //------- Compute mouvement for right diagonal -------//
    float theta_1r = asin((2*d_r + LEG_L*sin(theta_r))/(2*LEG_R)) + theta_r;
    float theta_2r = asin((2*d_r - LEG_L*sin(theta_r))/(2*LEG_R)) - theta_r;
    
    // --> Conversion for digitalisation
    float digit_theta_1r = 1500 - slope * theta_1r; // For COXA_FL
    float digit_theta_2r = 1500 + slope * theta_2r; // For COXA_BR

    float _digit_theta_1r = 1500 - slope * (-1 * theta_1r); // For COXA_FL   --> Because we have a symetrical mvmnt
    float _digit_theta_2r = 1500 + slope * (-1 * theta_2r); // For COXA_BR

    if(digit_theta_1r > ticks_max){digit_theta_1r = ticks_max;}
    if(digit_theta_1r < ticks_min){digit_theta_1r = ticks_min;}
    if(digit_theta_2r > ticks_max){digit_theta_2r = ticks_max;}
    if(digit_theta_2r < ticks_min){digit_theta_2r = ticks_min;}

    if(_digit_theta_1r > ticks_max){_digit_theta_1r = ticks_max;}
    if(_digit_theta_1r < ticks_min){_digit_theta_1r = ticks_min;}
    if(_digit_theta_2r > ticks_max){_digit_theta_2r = ticks_max;}
    if(_digit_theta_2r < ticks_min){_digit_theta_2r = ticks_min;}




    //if(( digit_theta_1r > ticks_max)||( digit_theta_1r < ticks_min)){ digit_theta_1r = ticks_middle;}
    //if(( digit_theta_2r > ticks_max)||( digit_theta_2r < ticks_min)){ digit_theta_2r = ticks_middle;}
    //if((_digit_theta_1r > ticks_max)||(_digit_theta_1r < ticks_min)){_digit_theta_1r = ticks_middle;}
    //if((_digit_theta_2r > ticks_max)||(_digit_theta_2r < ticks_min)){_digit_theta_2r = ticks_middle;}



    //------- Compute mouvement for left diagonal -------//
    float theta_1l = asin((2*d_l + LEG_L*sin(theta_r))/(2*LEG_R)) + theta_r;
    float theta_2l = asin((2*d_l - LEG_L*sin(theta_r))/(2*LEG_R)) - theta_r;
    
    // --> Conversion for digitalisation
    float digit_theta_1l = 1500 - slope * theta_1l; // For COXA_BL
    float digit_theta_2l = 1500 + slope * theta_2l; // For COXA_FR

    float _digit_theta_1l = 1500 - slope * (-1 * theta_1l); // For COXA_BL
    float _digit_theta_2l = 1500 + slope * (-1 * theta_2l); // For COXA_FR

    //if(( digit_theta_1l > ticks_max)||( digit_theta_1l < ticks_min)){ digit_theta_1l = ticks_middle;}
    //if(( digit_theta_2l > ticks_max)||( digit_theta_2l < ticks_min)){ digit_theta_2l = ticks_middle;}
    //if((_digit_theta_1l > ticks_max)||(_digit_theta_1l < ticks_min)){_digit_theta_1l = ticks_middle;}
    //if((_digit_theta_2l > ticks_max)||(_digit_theta_2l < ticks_min)){_digit_theta_2l = ticks_middle;}

    if(digit_theta_1l > ticks_max){digit_theta_1l = ticks_max;}
    if(digit_theta_1l < ticks_min){digit_theta_1l = ticks_min;}
    if(digit_theta_2l > ticks_max){digit_theta_2l = ticks_max;}
    if(digit_theta_2l < ticks_min){digit_theta_2l = ticks_min;}

    if(_digit_theta_1l > ticks_max){_digit_theta_1l = ticks_max;}
    if(_digit_theta_1l < ticks_min){_digit_theta_1l = ticks_min;}
    if(_digit_theta_2l > ticks_max){_digit_theta_2l = ticks_max;}
    if(_digit_theta_2l < ticks_min){_digit_theta_2l = ticks_min;}
    
    
    
    //------- Mouvement parameters -------//
    int air_step   = 22;   // Good values: air --> 25; floor --> 15
    int floor_step = 18;   // Good values: air --> 22; floor --> 18

    int step = air_step;

    int min = 1200;
    int max = 2000;
    int middle = 1500;

    //---------------------------- Mouvement for right side ----------------------------//
    for (int us = middle; us <= max; us += air_step) {               //Rise the legs
        pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick)); 
        
        sleep_ms(10);
    }

    // Transition for mouvement:
    float start_us1r = _digit_theta_1r;
    float stop_us1r  = digit_theta_1r;
    float slope_us1r = (stop_us1r - start_us1r);
    
    float start_us2r = _digit_theta_2r; 
    float stop_us2r  = digit_theta_2r;
    float slope_us2r = (stop_us2r - start_us2r);

    if(initialize){ 
        start_us1r = 1500; 
        start_us2r = 1500; 

        slope_us1r = (stop_us1r - start_us1r);
        slope_us2r = (stop_us2r - start_us2r);

        initialize = false;
    }

    for (int i = 0; i <= floor_step;  ++i){ 
    
        int us1r = start_us1r + i*slope_us1r/(floor_step*1.0);
        int us2r = start_us2r + i*slope_us2r/(floor_step*1.0);

        pca_write_pwm(COXA_FL, 0, to_ticks_us(us1r, us_par_tick));
        pca_write_pwm(COXA_BR, 0, to_ticks_us(us2r, us_par_tick));  

        sleep_ms(10);
    }

    for (int us = max; us >= middle; us -= air_step) {              //Get the legs down
        pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));  
        
        sleep_ms(10);
    }

    start_us1r = digit_theta_1r;
    stop_us1r  = 1500;
    slope_us1r = (stop_us1r - start_us1r);
    
    start_us2r = digit_theta_2r;
    stop_us2r  = 1500;
    slope_us2r = (stop_us2r - start_us2r);

    float start_us1l = 1500;
    float stop_us1l  = _digit_theta_1l;
    float slope_us1l = (stop_us1l - start_us1l);
    
    float start_us2l = 1500;
    float stop_us2l  = _digit_theta_2l;
    float slope_us2l = (stop_us2l - start_us2l);

    for (int i = 0; i <= floor_step;  ++i){ 

        int us1r = start_us1r + i*slope_us1r/(floor_step*1.0);
        int us2r = start_us2r + i*slope_us2r/(floor_step*1.0);

        int us1l = start_us1l + i*slope_us1l/(floor_step*1.0);
        int us2l = start_us2l + i*slope_us2l/(floor_step*1.0);

        pca_write_pwm(COXA_FL, 0, to_ticks_us(us1r, us_par_tick));
        pca_write_pwm(COXA_BR, 0, to_ticks_us(us2r, us_par_tick));

        pca_write_pwm(COXA_BL, 0, to_ticks_us(us1l, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us2l, us_par_tick));

        sleep_ms(10);
    }




    //---------------------------- Mouvement for Left side -----------------------------//
    for (int us = middle; us <= max; us += air_step) {               //Rise the legs
        pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick)); 
        
        sleep_ms(10);
    }

    // Transition for mouvement:

    start_us1l = _digit_theta_1l;
    stop_us1l  = digit_theta_1l;
    slope_us1l = (stop_us1l - start_us1l);
    
    start_us2l = _digit_theta_2l;
    stop_us2l  = digit_theta_2l;
    slope_us2l = (stop_us2l - start_us2l);

    for (int i = 0; i <= floor_step;  ++i){

        int us1l = start_us1l + i*slope_us1l/(floor_step*1.0);
        int us2l = start_us2l + i*slope_us2l/(floor_step*1.0);

        pca_write_pwm(COXA_BL, 0, to_ticks_us(us1l, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us2l, us_par_tick));

        sleep_ms(10);
    }

    for (int us = max; us >= middle; us -= air_step) {              //Get the legs down
        pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));  

        sleep_ms(10);
    }

    start_us1r = 1500;
    stop_us1r  = _digit_theta_1r;
    slope_us1r = (stop_us1r - start_us1r);
    
    start_us2r = 1500;
    stop_us2r  = _digit_theta_2r;
    slope_us2r = (stop_us2r - start_us2r);
    
    ////

    start_us1l = digit_theta_1l;
    stop_us1l  = 1500;
    slope_us1l = (stop_us1l - start_us1l);
    
    start_us2l = digit_theta_2l;
    stop_us2l  = 1500;
    slope_us2l = (stop_us2l - start_us2l);

    for (int i = 0; i <= floor_step;  ++i){ 

        int us1r = start_us1r + i*slope_us1r/(floor_step*1.0);
        int us2r = start_us2r + i*slope_us2r/(floor_step*1.0);

        int us1l = start_us1l + i*slope_us1l/(floor_step*1.0);
        int us2l = start_us2l + i*slope_us2l/(floor_step*1.0);

        pca_write_pwm(COXA_FL, 0, to_ticks_us(us1r, us_par_tick));
        pca_write_pwm(COXA_BR, 0, to_ticks_us(us2r, us_par_tick));

        pca_write_pwm(COXA_BL, 0, to_ticks_us(us1l, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us2l, us_par_tick));

        sleep_ms(10);

    }
}

void turn_without_moving(int angle_us){
    if(angle_us >= 1800){ angle_us = 1800; }
    if(angle_us <= 1200){ angle_us = 1200; }

    pca_write_pwm(COXA_FL, 0, to_ticks_us(angle_us, us_par_tick));
    pca_write_pwm(COXA_FR, 0, to_ticks_us(angle_us, us_par_tick));
    pca_write_pwm(COXA_BL, 0, to_ticks_us(angle_us, us_par_tick));
    pca_write_pwm(COXA_BR, 0, to_ticks_us(angle_us, us_par_tick));
}

void get_cmd(char *buffer, size_t size) {
    size_t index = 0;

    while (true){
        int c = getchar();

        if (c == '\r') {
            continue;                 // ignore CR
        }

        if (c == '\n') {
            break;                    // fin de ligne
        }

        if (index < size - 1) {
            buffer[index++] = (char)c;
        }
    }

    buffer[index] = '\0';
}

void print_help(int mode){
    if(mode == 0){
        printf("----------------------------------------------------------------------------------------\n");
        printf("You are in the initial mode. You can have access to the following submodes: \n");
        printf("- Servo control --> type servo_ctrl\n");
        printf("----------------------------------------------------------------------------------------\n");
    }

    if(mode == 1){
        printf("----------------------------------------------------------------------------------------\n");
        printf("You are in the servo ctrl mode. You can affect PWM to each servomotor individually\n");
        printf("You need to enter the channel of the servo you want to control. For example: COXA_FL\n");
        printf("If you want to go back to the main menu - type back\n");
        printf("----------------------------------------------------------------------------------------\n");
    }

    if(mode == 2){
        printf("----------------------------------------------------------------------------------------\n");
        printf("You chose to control servo motor on channel xxx. \n");
        printf("You are expected to enter a value between 1000 and 2000 - values of ticks\n");
        printf("If you want to go back to channel selection - type back \n");
        printf("----------------------------------------------------------------------------------------\n");
    } 
}

int handle_back(int mode){
    if(mode == 1){
        printf("----------------------------------------------------------------------------------------\n");
        printf("Moved to initial mode\n");
        printf("----------------------------------------------------------------------------------------\n");
        return 0;
    }

    if(mode == 2){
        printf("----------------------------------------------------------------------------------------\n");
        printf("Moved to channel selection mode\n");
        printf("----------------------------------------------------------------------------------------\n");
        return 1;
    }

    return 0;
}

int channel_selection(char *cmd){
    if(strcmp(cmd, "COXA_FL") == 0){
        return COXA_FL;
    
    }else if(strcmp(cmd, "COXA_FR") == 0){
        return COXA_FR;
    
    }else if(strcmp(cmd, "COXA_BL") == 0){
        return COXA_BL;
    
    } else if(strcmp(cmd, "COXA_BR") == 0){
        return COXA_BR;
    
    } else if(strcmp(cmd, "TROC_FL") == 0){
        return TROC_FL;
    
    } else if(strcmp(cmd, "TROC_FR") == 0){
        return TROC_FR;
    
    } else if(strcmp(cmd, "TROC_BL") == 0){
        return TROC_BL;
    
    } else if(strcmp(cmd, "TROC_BR") == 0){
        return TROC_BR;
    }else{
        return INVALID_CHANNEL;
    }   
}




void demo(int mode){
    if(mode == 0){ //Mini demo up down and quick look around!

        int us = 1500;
        pca_write_pwm(4, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(5, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(9, 0, to_ticks_us(us, us_par_tick));
        
        pca_write_pwm(2, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(3, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(12, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(13, 0, to_ticks_us(us, us_par_tick));

        //uint8_t channel = 13;

        int step = 5;
        int us_min = 1100;
        int us_max = 1900;


        for (int us = 1500; us >= us_min; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(2, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(3, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(12, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(13, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = us_min; us <= us_max; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(2, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(3, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(12, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(13, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = us_max; us >= 1500; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(2, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(3, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(12, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(13, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }


        //////////////////////////////// TRANSITION ///////////////////////////////////
        for (int us = 1500; us <= us_max; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(4, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(5, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(9, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = us_max; us >= us_min; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(4, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(5, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(9, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = us_min; us <= 1500; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(4, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(5, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(9, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }
    }

    if(mode == 1){ //Calibration
        int us_troca = 1500;
        int us_coxa  = 1500;
        
        pca_write_pwm(COXA_FL, 0, to_ticks_us(us_coxa, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us_coxa, us_par_tick));
        pca_write_pwm(COXA_BL, 0, to_ticks_us(us_coxa, us_par_tick));
        pca_write_pwm(COXA_BR, 0, to_ticks_us(us_coxa, us_par_tick));
        
            
        
        pca_write_pwm(TROC_BL, 0, to_ticks_us(us_troca, us_par_tick));
        pca_write_pwm(TROC_BR, 0, to_ticks_us(us_troca, us_par_tick));
        pca_write_pwm(TROC_FL, 0, to_ticks_us(us_troca, us_par_tick));
        pca_write_pwm(TROC_FR, 0, to_ticks_us(us_troca, us_par_tick));
    }

    if(mode == 2){ //Rise legs
        if(initialize){
            int us = 1500;
        
            pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));


            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));
            initialize = false;
        }

        int step = 10;
        /*
        for(int us = 1500; us <= 2000; us += step){
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);
        }

        for(int us = 2000; us >= 1500; us -= step){
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);
        }
        */

        pca_write_pwm(TROC_BL, 0, to_ticks_us(2000, us_par_tick));
        pca_write_pwm(TROC_FR, 0, to_ticks_us(2000, us_par_tick));

        
        
        /*
        for(int us = 1500; us <= 2000; us += step){
            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);
        }

        for(int us = 2000; us >= 1500; us -= step){
            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);
        }
        */
    }
    
    if(mode == 3){ // Spin around
        int step = 15;
        if(initialize){
            int us = 1500;
            pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));

            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));

            us = 1100;

            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));

            initialize = false;
        }
                    
        
        for (int us = 1500; us <= 2000; us += step) { // 2 et 12 montent
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 1000; us <= 1500; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }
        
        for (int us = 1500; us >= 1000; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 2000; us >= 1500; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 1500; us <= 2000; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 1000; us <= 1500; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 1500; us >= 1000; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 2000; us >= 1500; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }
        
    }

    if(mode == 4){
        int step = 30;
        int max = 1600;
        int min = 1100;
        if(initialize){
            int us = 1500;
            pca_write_pwm(4, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(5, 0, to_ticks_us(us, us_par_tick));
        
            pca_write_pwm(2, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(12, 0, to_ticks_us(us, us_par_tick));

            pca_write_pwm(3, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(13, 0, to_ticks_us(us, us_par_tick));

            us = min;
            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
            
            us = max;
            pca_write_pwm(9, 0, to_ticks_us(us, us_par_tick));

            initialize = false;
        }
                    
        
        for (int us = 1500; us <= 2000; us += step) { // 2 et 12 montent
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(2, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(12, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = min; us <= 1500; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));

            pca_write_pwm(9, 0, to_ticks_us(3000 - us, us_par_tick));
            

            sleep_ms(10);   
        }
        
        for (int us = 1500; us >= min; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(4, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(5, 0, to_ticks_us(3000 - us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 2000; us >= 1500; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(2, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(12, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 1500; us <= 2000; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(3, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(13, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = min; us <= 1500; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);
            pca_write_pwm(4, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(5, 0, to_ticks_us(3000 - us, us_par_tick));
            

            sleep_ms(10);   
        }

        for (int us = 1500; us >= min; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(9, 0, to_ticks_us(3000 - us, us_par_tick));
            

            sleep_ms(10);   
        }

        for (int us = 2000; us >= 1500; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(3, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(13, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }
        
    }

    if(mode == 5){
        int step = 10;
        int min = 1100;
        int max = 3000 - min;

        if(initialize){
            int us = 1500;
            pca_write_pwm(4, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(5, 0, to_ticks_us(us, us_par_tick));

            pca_write_pwm(3, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(13, 0, to_ticks_us(us, us_par_tick));

            us = 1600;
            pca_write_pwm(2, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(12, 0, to_ticks_us(us, us_par_tick));

            us = min;
            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
            
            us = max;
            pca_write_pwm(9, 0, to_ticks_us(us, us_par_tick));

            initialize = false;
        }
                    

        for (int us = min; us <= 1500; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(9, 0, to_ticks_us(3000 - us, us_par_tick));

            
            pca_write_pwm(5, 0, to_ticks_us(2600 - us, us_par_tick));
            pca_write_pwm(4, 0, to_ticks_us(400 + us, us_par_tick));

            sleep_ms(10);   
        }

        sleep_ms(1000);

        for (int us = 1500; us >= min; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(8, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(9, 0, to_ticks_us(3000 - us, us_par_tick));

            
            pca_write_pwm(5, 0, to_ticks_us(2600 - us, us_par_tick));
            pca_write_pwm(4, 0, to_ticks_us(400 + us, us_par_tick));

            sleep_ms(10);   
        }

        sleep_ms(1000);

        
    }

    if(mode == 6){ //Move forward
        //int air_step   = 25;
        //int floor_step = 15;

        int air_step   = 25;
        int floor_step = 20;

        int step = air_step;

        int min = 1300;
        int max = 3000 - min;

        if(initialize){
            int us = 1500;
            pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));

            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));

            us = 1500; //Here
            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));

            us = min;
            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));
            
            us = max;
            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));

            initialize = false;
        }
                    
        step = floor_step;
        for (int us = min; us <= 1500; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BR, 0, to_ticks_us(3000 - us, us_par_tick));

            if(min == 1300){
                pca_write_pwm(COXA_FR, 0, to_ticks_us((1500 - min)  + 2600 - us, us_par_tick));   // If min == 1300
                pca_write_pwm(COXA_BL, 0, to_ticks_us(-(1500 - min) + 400 + us, us_par_tick));
            }

            if(min == 1100){
                pca_write_pwm(COXA_FR, 0, to_ticks_us(2600 - us, us_par_tick));    // If min = 1100
                pca_write_pwm(COXA_BL, 0, to_ticks_us(400 + us, us_par_tick));
            }
            
            sleep_ms(10);   
        }

        step = air_step;

        for (int us = 1500; us <= 2000; us += step) { //Here
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = min; us <= max; us += step) {

            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BL, 0, to_ticks_us(3000 - us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 2000; us >= 1500; us -= step) { //Here
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }
        
        step = floor_step;
        for (int us = 1500; us >= min; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FL, 0, to_ticks_us(3000 - us, us_par_tick));

            
            if(min == 1300){
                pca_write_pwm(COXA_BL, 0, to_ticks_us((1500 - min)  + 2600 - us, us_par_tick));   // If min == 1300
                pca_write_pwm(COXA_FR, 0, to_ticks_us(-(1500 - min) + 400 + us, us_par_tick));
            }

            if(min == 1100){
                pca_write_pwm(COXA_BL, 0, to_ticks_us(2600 - us, us_par_tick));    // If min = 1100
                pca_write_pwm(COXA_FR, 0, to_ticks_us(400 + us, us_par_tick));
            }

            sleep_ms(10);   
        }
        
        step = air_step;
        
        for (int us = 1500; us <= 2000; us += step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = min; us <= max; us += step) {

            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FL, 0, to_ticks_us(3000 - us, us_par_tick));

            sleep_ms(10);   
        }

        for (int us = 2000; us >= 1500; us -= step) {
            //uint16_t ticks = to_ticks_us(us, us_par_tick);

            pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);   
        }
        
    }

    if(mode == 7){ // Identification of channels
        int step = 10; 
        int channel = 11;
        int channel_1 = 0;

        pca_write_pwm(channel_1, 0, to_ticks_us(1500, us_par_tick));
        
        
        for (int us = 2000; us >= 1000; us -= step) {
            pca_write_pwm(channel, 0, to_ticks_us(us, us_par_tick));
            sleep_ms(10);   
        }

        sleep_ms(1000);
        
        for (int us = 1000; us <= 2000; us += step) {
            pca_write_pwm(channel, 0, to_ticks_us(us, us_par_tick));
            sleep_ms(10);   
        }
        sleep_ms(1000);
        

        
    }

    if(mode == 8){ // Movement with smooth transitions between translation and rotation
        bool incr_angle = true;
        bool modif_angle = true;
        bool back_to_zero_angle = false;

        bool incr_dist = true;
        bool modif_dist = true;
        

        int angle = 0;
        int step_angle = 2;


        float D = 0;
        float step_d = 0.25;

        float D_max = 4;
        float D_min = 0;

        float Angle_max = 20;
        float Angle_min = -20;

        while (true) {

            for(int i = 0; i < 1; ++i){
                move(D, 0, angle*M_PI/180);
            }

            if(modif_dist){
                if(incr_dist){
                    D += step_d;
                    if(D > D_max){D -= 2*step_d; incr_dist = false;}
                }else{
                    D -= step_d;
                    if(D < D_min){D = D_min; incr_dist = true; modif_dist = false;}
                }
            }else{
                if(incr_angle){
                    angle += step_angle;

                    if(angle > Angle_max){angle -= 2*step_angle; incr_angle = false;}
                    if((back_to_zero_angle)&&(angle > 0)){angle = 0; modif_dist = true; back_to_zero_angle = false;}

                }else{
                    angle -= step_angle;
                    if(angle < Angle_min){angle = Angle_min; incr_angle = true; back_to_zero_angle = true;}
                }
            }    
        }
    }

    if(mode == 9){ // Look around without translating
        int us_troca = 1500;
        int us_coxa  = 1500;
        int step = 3;
        
        pca_write_pwm(COXA_FL, 0, to_ticks_us(us_coxa, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us_coxa, us_par_tick));
        pca_write_pwm(COXA_BL, 0, to_ticks_us(us_coxa, us_par_tick));
        pca_write_pwm(COXA_BR, 0, to_ticks_us(us_coxa, us_par_tick));

        pca_write_pwm(TROC_BL, 0, to_ticks_us(us_troca, us_par_tick));
        pca_write_pwm(TROC_BR, 0, to_ticks_us(us_troca, us_par_tick));
        pca_write_pwm(TROC_FL, 0, to_ticks_us(us_troca, us_par_tick));
        pca_write_pwm(TROC_FR, 0, to_ticks_us(us_troca, us_par_tick));

        for(int us = 1500; us <= 1800; us += step){

            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));
            
            sleep_ms(10);

        }

        for(int us = 1800; us >= 1200; us -= step){

            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));
            
            sleep_ms(10);
            
        }

        for(int us = 1200; us <= 1500; us += step){

            pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));
            pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));

            sleep_ms(10);    
        }
        
            
        
        //pca_write_pwm(TROC_BL, 0, to_ticks_us(us_troca, us_par_tick));
        //pca_write_pwm(TROC_BR, 0, to_ticks_us(us_troca, us_par_tick));
        //pca_write_pwm(TROC_FL, 0, to_ticks_us(us_troca, us_par_tick));
        //pca_write_pwm(TROC_FR, 0, to_ticks_us(us_troca, us_par_tick)); 
    }

    if(mode == 10){ // Interactive callibration
        
        int servo_channel = INVALID_CHANNEL;
        int ticks = 1000;
        
        int mode = 0;
        char cmd[64];

        while(true){
            
            
            get_cmd(cmd, sizeof(cmd));
            printf("----------------------------------------------------------------------------------------\n");
            printf("Command: %s\n", cmd);
            printf("----------------------------------------------------------------------------------------\n");

            
            
            
            if (strcmp(cmd, "help") == 0){
                print_help(mode);

            }else if(strcmp(cmd, "back") == 0){
                mode = handle_back(mode);

            }
            
            
            else if(mode == 0){ //Initial mode
                if (strcmp(cmd, "servo_ctrl") == 0){
                    printf("Enter servo channel\n");

                    mode = 1;    
                }

            }else if(mode == 1){ //Waiting for servo channel
                servo_channel = channel_selection(cmd);

                if(servo_channel != INVALID_CHANNEL){
                    printf("Enter ticks - Between 1000 and 2000\n");
                    mode = 2;
                }else{
                    printf("Invalid servo channel\n");
                } 
                
            } else if (mode == 2){ //Waiting for servo ticks
                long staff_value = strtol(cmd, NULL, 10);
                ticks = (int)staff_value;

                if((ticks >= 1000)&&(ticks <= 2000)){
                    pca_write_pwm(servo_channel, 0, to_ticks_us(ticks, us_par_tick));
                }else{
                    printf("Invalid value for ticks\n");
                }
            }  
            
        }
        
    }
}



void recenter(int previous_angle_us){
    int troca_step = 10;
    int coxa_step = 20;

    float start = previous_angle_us;
    float stop = 1500;
    float slope_us = (stop-start);

    for(int us = 1500; us <= 2000; us+= troca_step){
        pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
        sleep_ms(10);
    }

    for(int i = 0; i < coxa_step; ++i){

        int us = start + (slope_us*i)/(coxa_step*1.0);

        pca_write_pwm(COXA_BL, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(COXA_FR, 0, to_ticks_us(us, us_par_tick));
        sleep_ms(10);
    }

    for(int us = 2000; us >= 1500; us -= troca_step){
        pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));
        sleep_ms(10);
    }

    ///
    
    for(int us = 1500; us <= 2000; us+= troca_step){
        pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));
        sleep_ms(10);
    }

    for(int i = 0; i < coxa_step; ++i){

        int us = start + (slope_us*i)/(coxa_step*1.0);

        pca_write_pwm(COXA_BR, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(COXA_FL, 0, to_ticks_us(us, us_par_tick));
        sleep_ms(10);
    }

    for(int us = 2000; us >= 1500; us -= troca_step){
        pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));
        sleep_ms(10);
    }
}

void wake_up(){
    int us_coxa = 1500;
    int us_troca = 2000;

    pca_write_pwm(COXA_FL, 0, to_ticks_us(us_coxa, us_par_tick));
    pca_write_pwm(COXA_FR, 0, to_ticks_us(us_coxa, us_par_tick));
    pca_write_pwm(COXA_BL, 0, to_ticks_us(us_coxa, us_par_tick));
    pca_write_pwm(COXA_BR, 0, to_ticks_us(us_coxa, us_par_tick));
    
    pca_write_pwm(TROC_BL, 0, to_ticks_us(us_troca, us_par_tick));
    pca_write_pwm(TROC_BR, 0, to_ticks_us(us_troca, us_par_tick));
    pca_write_pwm(TROC_FL, 0, to_ticks_us(us_troca, us_par_tick));
    pca_write_pwm(TROC_FR, 0, to_ticks_us(us_troca, us_par_tick));

    sleep_ms(1000);

    int step = 10;

    for(int us = 2000; us >= 1500; us -= step){
        pca_write_pwm(TROC_BL, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_BR, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_FL, 0, to_ticks_us(us, us_par_tick));
        pca_write_pwm(TROC_FR, 0, to_ticks_us(us, us_par_tick));

        sleep_ms(10);
    }

}

