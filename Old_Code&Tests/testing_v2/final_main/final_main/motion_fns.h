// header guard
#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

// extern int motor_speed = 200;
// extern int duration_steer = 1400; // require testing to determine value
// extern int duration_start_forward = 1400;
// extern Servo servo_claw;

/** variables */ 
// Pins Set-up:
extern const int hall_pn = 8;
extern const int button_pn = 5;
extern const int l0_pn = 2; //left
extern const int l1_pn = 3; //right
extern const int l2_pn = 4; //far right (for juntion counting)
extern const int us1E_pn = 6; // yellow wire
extern const int us1T_pn = 7; // green wire
extern const int ledG_pn = A2; // now analog
extern const int ledR_pn = A3; // now analog
extern const int ledA_pn = 10;
extern const int servo1_pn = 9;
extern const int us2E_pn = 12;
extern const int us2T_pn = 13;

// Sensor Values
extern int l0, l1, l2, hall;
extern int loop_count = 0;
extern float us1_distance = 200.0;
extern float us2_distance = 200.0;

// button debounce variables
extern unsigned long currentTime = 0;
extern byte lastButtonState = LOW;
extern unsigned long debounceDuration = 50; // millis
extern unsigned long lastTimeButtonStateChanged = 0;

// time variable
extern unsigned long started_time = 0;
extern unsigned long return_home_time = 0;
extern int return_home_duration = 5200;

// PID tunnel navigation variables
extern float Kp = 10; // related to the proportional control term;
extern float P;
extern float distance_tunnel1 = 5;
extern float distance_tunnel2 = 7; //unit in cm

const float maxspeedR = 200.0;
const float maxspeedL = 200.0;
const float basespeedR = 150.0;
const float basespeedL = 150.0;


/*************
   flags
**************/
bool flag_onoff = false; // push button: robot on or off
bool flag_ledA = false;
bool flag_started;       // has robot completed initial start?
char flag_nav = 'P';
//"P": stop
//"L": adjust left
//"R": adjust right
//"B": backwards
//"F": forwards
bool flag_blk = false; // block collected
bool flag_delivered = false;
bool flag_tunnel; // true if in tunnel, false if not
int box_intend = 1;
int box_pass = 0;
bool flag_box_register;

// void stop_move();
// void move_forward();
// void move_backward();
// void adjust_left();
// void adjust_right();
// void turn_90left();
// void turn_90right();
// void turn_180();
// void tunnel_PID_control(float distance_wall);
void blk_find();
void line_follow();
void start_route();
void blk_magnet();
void blk_LEDindication();
void blk_collect();
void blk_delivery();
void blk_retriet();
void return_home();
void us1_measure();
void us2_measure();
void tunnel_PID_control();
#endif
