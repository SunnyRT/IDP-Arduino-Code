// header guard
#ifndef start_H
#define start_H

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <Arduino.h>
/** Run this once at the start to initialise flags & set-up variables */

// Digital pins 1 & 2 were avoided on the advice of the technicians

const int ledG_pn = A2; // analog, so analogRead is required
const int ledR_pn = A3; // now analog
const int l0_pn = 2; //left
const int l1_pn = 3; //right
const int l2_pn = 4; //far right (for juntion counting)
const int button_pn = 5;
const int us1E_pn = 6; // front ultrasonic sensor: echo
const int us1T_pn = 7; // front ultrasonic sensor: trigger
const int hall_pn = 8;
const int servo1_pn = 9;
const int ledA_pn = 10;
const int us2E_pn = 12; // left ultrasonic sensor: echo
const int us2T_pn = 13; // left ultrasonic sensor: trigger

int motor_speed = 200;
int duration_steer = 1400; // time for which turning occurs
int duration_start_forward = 1400;

// Sensor Values
int l0, l1, l2, hall;
int loop_count = 0;
float us1_distance = 200.0; // initialised to far away, so that the tunnel is not immediately triggered if first value is erroneously small
float us2_distance = 200.0;

// button debounce variables
unsigned long currentTime = 0;  // keep count of current time
byte lastButtonState = LOW;     // record button state to handle debouncing
unsigned long debounceDuration = 50; // millis between debouncing
unsigned long lastTimeButtonStateChanged = 0;

// time variables
unsigned long started_time;     
unsigned long return_home_time = 0;
int return_home_duration = 6500;

bool flag_onoff = false; // push button: robot on or off
bool flag_ledA = false;  // whether amber LED should be flashing or not
bool flag_started;       // has robot completed initial start?
char flag_nav = 'P';     // initialise to stopped
//"P": stop
//"L": adjust left
//"R": adjust right
//"B": backwards
//"F": forwards
bool flag_blk = false; // block collected
bool flag_delivered = false; // true when block has been deposited in a box
int flag_tunnel = 0; //
/*
   0: before the tunnel --> us2 reading every 5 loops
   1: within the tunnel --> us2 reading every loops
   2: after the tunnel --> no us2 reading
   this allowed us to not have to read the ultrasonic sensor more than necessary,
   as it did slow the loop down
*/
int box_intend = 1;
int box_pass = 0;
bool flag_box_register; // has a junction been registered
// desired distance to maintain to tunnel walls
float distance_tunnel1 = 5.5; // outgoing journey
float distance_tunnel2 = 7;   // return journey

/* function declarations */

void stop_move();
void tunnel_P_control(float distance_wall);
void move_backward();
void turn_90left();
void turn_90right();
void turn_180();
void move_forward();
void trap_blk();
void release_blk();


#endif