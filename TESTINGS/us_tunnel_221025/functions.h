// header guard
#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>


// Global Variables ===================================================

extern const int ldr_pn, hall_pn, ir1_pn, ir2_pn, button_pn;
extern const int l0_pn, l1_pn, l2_pn, l3_pn, us1E_pn, us1T_pn;
extern const int ledG_pn, ledA_pn, ledR_pn, servo1_pn;
extern const int us2E_pn, us2T_pn;

extern int ldr, l0, l1, l2, ir1, ir2, hall, push;
extern float us1_distance, us2_distance;
extern float ir1_avg, ir2_avg, us1_avg, us2_avg;

extern int motor_speed;
extern int duration_steer; // require testing to determine value

// button debounce variables
extern byte lastButtonState;
extern unsigned long debounceDuration; // millis
extern unsigned long lastTimeButtonStateChanged;
extern bool onoff;

//ledA_flash variables
extern int ledAState;  // ledState used to set the LED
extern unsigned long previousMillis;  // will store last time LED was updated
extern const long interval;  // 2hz --> 0.5 second interval at which to blink (milliseconds)


// flags
extern int count;
extern bool flag_onoff;
extern bool flag_started;
extern bool flag_line;
extern char flag_nav; 
extern int flag_side; 

//=============================================================

// global
void sensor_read();
float us_measure();
float moving_avg();
bool update_onoff();
void line_follow();
void ledA_flash();
void stop_move();
void move_forward();
void move_backward();
void adjust_left();
void adjust_right();
void turn_90left();
void turn_90right();

// side == 0;
void start_route();

// side == 1;
void ramp_up();
void ramp_down();

// side == 2;
void blk_magnet_identify();
void blk_collect();


// side == 3;
void tunnel();

// side == 4;
void box_find();

void box_delivery();

#endif
