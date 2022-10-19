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

// calculate distance from ultrasonic sensors
float us_measure(trig_pin, echo_pin){
    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
     // measure duration of pulse from ECHO pin
    float duration_us = pulseIn(echo_pin, HIGH);
    // return distance (*v_sound /2)
    return duration_us * 0.017;
}

// collects sensor readings (will be run every loop)
void sensor_read(){
  // light/line sensors:
    ldr = analogRead(ldr_pn);
    hall = analogRead(hall_pn);
    ir1 = analogRead(ir1_pn);
    ir2 = analogRead(ir2_pn);
    push = digitalRead(push_pn);
    l0 = digitalRead(l0_pn);
    l1 = digitalRead(l1_pn);
    l2 = digitalRead(l2_pn);
    l3 = digitalRead(l3_pn);
    us1_distance = us_measure(us1T_pn, us1E_pn);
    us2_distance = us_measure(us2T_pn, us2E_pn); 
};

// moving average function
void update_avg(avg_array, avg_value, new_value){
    float sum_vals=0;
    int length = 10;
    // array structure: 
    // avg_array[0]=latest
    // avg_array[length]=oldest

    // remove oldest(last) item in array
    for(int j=1; j<(length-1); j++){

        // move all values along (to the right) by one
        avg_array[j]=avg_array[j-1]; 
    } 
    // add newest value to start of list
    avg_array[0] = new_value;
    // find new average
    for(int i=0; i<length; i++){ // 10 = array length
        sum_vals += avg_array[i]
    }
    avg_value = sum_vals / length; //10=array length
    return avg_value
}


void line_follow();
void move_forward()
{
    flag_foward = true;
    flag_Lturn = false;
    flag_Rturn = false;
    flag_stop = false;
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(150);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(150);
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

void turn_90left(){
    Rwheel->run(FORWARD);
    Rwheel->setSpped
};
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




// how to read from different sensors
void IR_read(){

}
