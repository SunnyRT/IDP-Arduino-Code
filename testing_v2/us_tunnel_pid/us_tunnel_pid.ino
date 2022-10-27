
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//side ultrasonic sensors
const int us2E_pn = 12; // yellow wire
const int us2T_pn = 13; // green wire
bool flag_nav;

// reference link: https://create.arduino.cc/projecthub/anova9347/line-follower-robot-with-pid-controller-cdedbd


/*************************************************************************
   setup the motor and create the DC motor object
 *************************************************************************/

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT


/*************************************************************************
   PID control system variables
 *************************************************************************/
float Kp = 1; // related to the proportional control term;
// change the value by trial-and-error (ex: 0.07).
float Ki = 0.1; // related to the integral control term;
// change the value by trial-and-error (ex: 0.0008).
float Kd = 0.1; // related to the derivative control term;
// change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

/*************************************************************************
   Global variables
 *************************************************************************/
int lastError = 0;
int us2_distance;


void setup() {
  pinMode(us2E_pn, INPUT);
  pinMode(us2T_pn, OUTPUT);
  Serial.begin(9600);

}



/*************************************************************************
   trigger conditions
 *************************************************************************/
void loop()
{
  us2_measure();
  Serial.print("us2: ");
  Serial.println(us2_distance);
  tunnel_PID_control();


};


/*************************************************************************
   functions
 *************************************************************************/
void us2_measure()
{
  digitalWrite(us2T_pn, LOW);
  delayMicroseconds(1);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(us2T_pn, HIGH);
  delayMicroseconds(10);
  digitalWrite(us2T_pn, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float us2_duration = pulseIn(us2E_pn, HIGH);
  // get distance values(*v_sound /2)
  us2_distance = us2_duration * 0.017;
}


void tunnel_PID_control()
{

  /*************************************************************************
    Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
  *************************************************************************/
  const int maxspeedR = 150;
  const int maxspeedL = 150;
  const int basespeedR = 100;
  const int basespeedL = 100;

  int error = 5.0 - us2_distance; // 5.0 is the ideal distance from the tunnel wall

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

  flag_nav = 'F';
//  Rwheel->run(FORWARD);
//  Rwheel->setSpeed(motorspeedR);
  Serial.print("speedR: ");
  Serial.println(motorspeedR);

//  Lwheel->run(FORWARD);
//  Lwheel->setSpeed(motorspeedL);
  Serial.print("speedL: ");
  Serial.println(motorspeedL);
}
