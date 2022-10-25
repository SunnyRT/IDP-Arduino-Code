#include "functions.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

const int us1E_pn=6; // yellow wire
const int us1T_pn=7; // green wire

// reference link: https://create.arduino.cc/projecthub/anova9347/line-follower-robot-with-pid-controller-cdedbd


/*************************************************************************
 * setup the motor and create the DC motor object
 *************************************************************************/

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT


/*************************************************************************
 * PID control system variables
 *************************************************************************/
float Kp = 0; // related to the proportional control term;
              // change the value by trial-and-error (ex: 0.07).
float Ki = 0; // related to the integral control term;
              // change the value by trial-and-error (ex: 0.0008).
float Kd = 0; // related to the derivative control term;
              // change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

/*************************************************************************
 * Global variables
 *************************************************************************/
int lastError = 0;
float us1_avg;
int side;


void setup(){
  
}



/*************************************************************************
 * trigger conditions
 *************************************************************************/
void loop()
{
    if (side == 5)
    {
        tunnel_PID_control();
    }
};


/*************************************************************************
 * functions
 *************************************************************************/
void tunnel_PID_control()
{
    
/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const int maxspeedR = 150;
const int maxspeedL = 150;
const int basespeedR = 100;
const int basespeedL = 100;

    int error = 5.0 - us1_avg; // 5.0 is the ideal distance from the tunnel wall

    P = error;
    I = I + error;
    D = error - lastError;
    lastError = error;
    int motorspeed = P * Kp + I * Ki + D * Kd; // calculate the correction
                                               // needed to be applied to the speed

    int motorspeedR = basespeedR + motorspeed;
    int motorspeedL = basespeedL - motorspeed;

    if (motorspeedR > maxspeedR)
    {
        motorspeedR = maxspeedR;
    }
    if (motorspeedL > maxspeedL)
    {
        motorspeedL = maxspeedL;
    }
    if (motorspeedR < 0)
    {
        motorspeedR = 0;
    }
    if (motorspeedL < 0)
    {
        motorspeedL = 0;
    }
    move_forward(motorspeedR, motorspeedL);
}

/**************************************************************************
* updated move_funward function for 2 different speed inputs
***************************************************************************/
void move_forward(int speedR, int speedL)
{
  flag_nav = 'F';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(speedR);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(speedL);
}
