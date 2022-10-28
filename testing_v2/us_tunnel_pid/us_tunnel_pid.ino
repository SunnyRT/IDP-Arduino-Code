
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//side ultrasonic sensors
const int us1E_pn = 6; // purple wire
const int us1T_pn = 7; // yellow wire
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
float Kp = 10; // related to the proportional control term;
// change the value by trial-and-error (ex: 0.07).
float Ki = 0.0; // related to the integral control term;
// change the value by trial-and-error (ex: 0.0008).
float Kd = 0.0; // related to the derivative control term;
// change the value by trial-and-error (ex: 0.6).
float P;
float I;
float D;
float distance_tunnel = 7;


/*************************************************************************
   Global variables
 *************************************************************************/
float lastError = 0;
float us1_distance;
float us2_distance;

/*************************************************************************
  Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const float maxspeedR = 250.0;
const float maxspeedL = 250.0;
const float basespeedR = 200.0;
const float basespeedL = 200.0;


void setup() {
  pinMode(us1E_pn, INPUT);
  pinMode(us1T_pn, OUTPUT);
  pinMode(us2E_pn, INPUT);
  pinMode(us2T_pn, OUTPUT);
  Serial.begin(9600);
  AFMS.begin(); //Connect to the Controller

}



/*************************************************************************
   trigger conditions
 *************************************************************************/
void loop()
{
  //  us1_measure();
  //  Serial.print("us1: ");
  //  Serial.println(us1_distance);
  us2_measure();
  Serial.print("us2: ");
  Serial.println(us2_distance);

  if (us2_distance > 12.0) {
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(200);
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(200);
    Serial.println("Wall");
  }
  else {
    tunnel_PID_control(distance_tunnel);
    Serial.println("tunnel");
  }



};


/*************************************************************************
   functions
 *************************************************************************/
void us1_measure()
{
  digitalWrite(us1T_pn, LOW);
  delayMicroseconds(1);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(us1T_pn, HIGH);
  delayMicroseconds(10);
  digitalWrite(us1T_pn, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float us1_duration = pulseIn(us1E_pn, HIGH);
  // get distance values(*v_sound /2)
  us1_distance = us1_duration * 0.017;
}

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


void tunnel_PID_control(float distance_ref)
{
  float error = distance_ref - us2_distance; // 5.0 is the ideal distance from the tunnel wall

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  float motorspeed = P * Kp + I * Ki + D * Kd; // calculate the correction
  // needed to be applied to the speed

  //  Serial.print("error: ");
  //  Serial.println(error);
  //  Serial.print("motorspeed: ");
  //  Serial.println(motorspeed);


  float motorspeedR = basespeedR - motorspeed;
  float motorspeedL = basespeedL + motorspeed;

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
      Rwheel->run(FORWARD);
      Rwheel->setSpeed(motorspeedR);
//  Serial.print("speedR: ");
//  Serial.println(motorspeedR);

      Lwheel->run(FORWARD);
      Lwheel->setSpeed(motorspeedL);
//  Serial.print("speedL: ");
//  Serial.println(motorspeedL);
}
