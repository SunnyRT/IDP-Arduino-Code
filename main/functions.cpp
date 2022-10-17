/* include all libraries */
#include "functions.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT

// global
void sersor_read(){
    l0 = digitalRead(light0);
    l1 = digitalRead(light1);
};

void line_follow();
void move_forward()
{
    flag_foward = true;
    flag_Lturn = false;
    flag_Rturn = false;
    flag_stop = false;
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(200);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(200);
}

void adjust_left()
{
    flag_foward = false;
    flag_Lturn = true;
    flag_Rturn = false;
    flag_stop = false;
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(150);
    Lwheel->run(BACKWARD);
    Lwheel->setSpeed(150);
}

void adjust_right()
{
    flag_foward = false;
    flag_Lturn = false;
    flag_Rturn = true;
    flag_stop = false;
    Rwheel->run(BACKWARD);
    Rwheel->setSpeed(150);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(150);
}

void stop_move()
{
    Rwheel->run(RELEASE);
    Rwheel->setSpeed(0);
    Lwheel->run(RELEASE);
    Lwheel->setSpeed(0);
    flag_foward = false;
    flag_Lturn = false;
    flag_Rturn = false;
    flag_stop = true;
}

void turn_90left();
void turn_90right();

// side == 0;
void start_route();

// side == 1;
void ramp_up();
void ramp_down();

// side == 2;
void blk_fn();

void blk_approach();
void blk_magnet_identify();
void blk_magnet_indicate();
void blk_collect();
void blk_retriet();

// side == 3;
void tunnel();

// side == 4;
void box_find();

void box_delivery();
