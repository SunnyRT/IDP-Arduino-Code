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
#define light1 3


int l0, l1;

bool flag_foward = false;
bool flag_Lturn = false;
bool flag_Rturn = false;
bool flag_stop = true;


void setup() {
  pinMode(light0, INPUT); //light sensor on the left
  pinMode(light1, INPUT); //light sensor on the right
  Serial.begin(9600);
  AFMS.begin(); //Connect to the controller
}

void loop() {
  //read and store light sensor values
  l0 = digitalRead(light0);
  l1 = digitalRead(light1);

  //print values of the light sensors to the serial monitor
  Serial.print("Sensor Readings: ");
  Serial.print(l0);
  Serial.println(l1);


  Serial.print("Flag States: ");
  Serial.print(flag_Lturn);
  Serial.print(flag_foward);
  Serial.println(flag_Rturn);

  //neither l0 or l1 detects the line
  if ((l0 == HIGH && l1 == HIGH) && flag_foward == false) {
    //forward
    move_forward();
    Serial.println("MOVE FORWARD!");
  }


  //l0 detects the line
  else if ((l0 == LOW && l1 == HIGH) && flag_Lturn == false) {
    //adjust left slightly
    adjust_left();
  }


  //l1 detects the line 
  else if ((l0 == HIGH && l1 == LOW) && flag_Rturn == false) {
    //adjust right slightly
    adjust_right();
  }

  //l0 and l1 both detect the line
  else {
    stop_move();
  }


}


//functions
void move_forward() {
    flag_foward = true;
    flag_Lturn = false;
    flag_Rturn = false;
    flag_stop = false;
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(200);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(200);
}

void adjust_left() {
    flag_foward = false;
    flag_Lturn = true;
    flag_Rturn = false;
    flag_stop = false;
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(150);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(150);
}

void adjust_right() {
    flag_foward = false;
    flag_Lturn = false;
    flag_Rturn = true;
    flag_stop = false;
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
  flag_foward = false;
  flag_Lturn = false;
  flag_Rturn = false;
  flag_stop = true;
}
