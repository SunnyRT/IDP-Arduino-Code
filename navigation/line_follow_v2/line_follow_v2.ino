/* include all libraries */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);  //RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);  //LEFT

/*defining pins and variables*/

#define l0_pin 2 // left 
#define l1_pin 3 // right


int l0, l1;

char flag_nav_line = "P"; 
//indicate whether the robot is moving forward / adjust left / adjust right / stop
// initialize flag_linestate to be "P": stop
//"P": stop
//"L": adjust left
//"R": adjust right
//"B": backwards
//"F": forwards


int motor_speed = 250;

void setup() {
  pinMode(l0_pin, INPUT); //light sensor on the left
  pinMode(l1_pin, INPUT); //light sensor on the right
  Serial.begin(9600);
  AFMS.begin(); //Connect to the controller
}

void loop() {
  //read and store light sensor values
  l0 = digitalRead(l0_pin);
  l1 = digitalRead(l1_pin);

  //print values of the light sensors to the serial monitor
  Serial.print("Sensor Readings: ");
  Serial.print(l0);
  Serial.println(l1);


  Serial.print("Flag States: ");
  Serial.print(flag_Lturn);
  Serial.print(flag_foward);
  Serial.println(flag_Rturn);

  //neither l0 or l1 detects the line
  if ((l0 == HIGH && l1 == HIGH) && flag_nav_line !== "F") {
    //forward
    move_forward();
    Serial.println("MOVE FORWARD!");
  }


  //l0 detects the line
  else if ((l0 == LOW && l1 == HIGH) && flag_nav_line !== "L") {
    //adjust left slightly
    adjust_left();
  }


  //l1 detects the line 
  else if ((l0 == HIGH && l1 == LOW) && flag_nav_line !== "R") {
    //adjust right slightly
    adjust_right();
  }

  //l0 and l1 both detect the line
  else if ((l0 == LOW && l1 == LOW) && flag_nav_line !== "P")  {
    stop_move();
  }


}


//functions
void move_forward() {
  flag_nav_line = "F";
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}

void adjust_left() {
  flag_nav_line = "L";
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
}

void adjust_right() {
  flag_nav_line = "R";
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);

}

void stop_move() {
  flag_nav_line = "P";
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);

}
