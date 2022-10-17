/* include all libraries */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);  //RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);  //LEFT

/*defining pins and variables*/

#define light0 2
//#define light1 3
#define light2 4


int l0, l2;

bool forward_flag = false;
bool backward_flag = false;
bool Lturn_flag = false;
bool Rturn_flag = false;
bool stop_flag = true;


void setup() {
  pinMode(light0, INPUT); //light sensor on the left
//  pinMode(light1, INPUT); //light sensor in the middle
  pinMode(light2, INPUT); //light sensor on the right
  Serial.begin(9600);
  AFMS.begin(); //Connect to the controller
}

void loop() {
  //read and store light sensor values
  l0 = digitalRead(light0);
//  l1 = digitalRead(light1);
  l2 = digitalRead(light2);

  //print values of the light sensors to the serial monitor
  Serial.print("Sensor Readings: ");
  Serial.print(l0);
//  Serial.print(l1);
  Serial.println(l2);


  Serial.print("Flag States: ");
  Serial.print(Lturn_flag);
  Serial.print(forward_flag);
  Serial.println(Rturn_flag);

  if ((l0 == HIGH && l2== HIGH) && forward_flag == false) {
    //forward
    forward_flag = true;
    Lturn_flag = false;
    Rturn_flag = false;
    stop_flag = false;
    move_forward();
    Serial.println("MOVE FORWARD!");
  }


  //l0 detects the line
  else if ((l0 == LOW && l2 == HIGH) && Lturn_flag == false) {
    //adjust left slightly
    forward_flag = false;
    Lturn_flag = true;
    Rturn_flag = false;
    stop_flag = false;
    adjust_left();
  }


    //l2 detects the line 
  else if ((l0 == HIGH && l2 == LOW) && Rturn_flag == false) {
    //adjust right slightly
    forward_flag = false;
    Lturn_flag = false;
    Rturn_flag = true;
    stop_flag = false;
    adjust_right();

  }

  



}


//functions
void move_forward() {
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(200);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(200);
}

void move_backward() {
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(100);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(100);
}

void adjust_left() {
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(150);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(150);
}

void adjust_right() {
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(150);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(150);

}

void stop_move() {
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);
  forward_flag = false;
  Lturn_flag = false;
  Rturn_flag = false;
  stop_flag = true;
}
