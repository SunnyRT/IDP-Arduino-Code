/* include all libraries */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT

/*defining pins and variables*/

#define l0_pin 3 // left
#define l1_pin 4 // right
#define l2_pin 5 // far right

int l0, l1, l2;

char flag_nav = 'P';
// indicate whether the robot is moving forward / adjust left / adjust right / stop
//  initialize flag_linestate to be "P": stop
//'P': stop
//'L': adjust left
//'R': adjust right
//'B': backwards
//'F': forwards

int motor_speed = 250;

void setup()
{
  pinMode(l0_pin, INPUT); // light sensor on the left
  pinMode(l1_pin, INPUT); // light sensor on the right
  pinMode(l2_pin, INPUT); // light sensor on the far right
  Serial.begin(9600);
  AFMS.begin(); // Connect to the controller
}

void loop()
{
  // read and store light sensor values
  l0 = digitalRead(l0_pin);
  l1 = digitalRead(l1_pin);
  l2 = digitalRead(l2_pin);

  // print values of the light sensors to the serial monitor
  Serial.print("Sensor Readings: ");
  Serial.print(l0);
  Serial.print(l1);
  Serial.println(l2);

  Serial.print("Flag States: ");
  Serial.println(flag_nav);

  // l2 detects the line --> junctions (cross or T)
  if (l2 == LOW && flag_nav != 'P')
  {
    Serial.println("Junctions detected!");
    stop_move();
    //    // report an error if left sensor detects the line but right sensor does not.
    //    if (l0 == LOW && l1 == HIGH) {
    //      Serial.println("Error.");
    //    }
    //
    //    // all 3 light sensors detect the line --> cross junction
    //    else if ((l0 == LOW && l1 == LOW) && flag_nav != "P") {
    //      Serial.println("Cross junction detected!");
    //      stop_move();
    //      //call relevant functions
    //    }
    //
    //    // only the 2 right sensors detetc the line --> T junction
    //    else if ((l0 ==HIGH && l1 == LOW) && flag_nav != "P") {
    //      Serial.println("T junction detected!");
    //      stop_move();
    //      //call relevant functions
    //    }
    //
    //
    //    else {
    //      //continue with whatever it is doing.
    //    }
  }

  else
  {
    // neither l0 or l1 detects the line
    if ((l0 == HIGH && l1 == HIGH) && flag_nav != 'F')
    {
      // forward
      move_forward();
      Serial.println("MOVE FORWARD!");
    }

    // l0 detects the line
    else if ((l0 == LOW && l1 == HIGH) && flag_nav != 'L')
    {
      // adjust left slightly
      adjust_left();
    }

    // l1 detects the line
    else if ((l0 == HIGH && l1 == LOW) && flag_nav != 'R')
    {
      // adjust right slightly
      adjust_right();
    }

    // l0 and l1 both detect the line
    else if ((l0 == LOW && l1 == LOW) && flag_nav != 'P')
    {
      // forward
      move_forward();
      Serial.println("MOVE FORWARD!");
    }
  }
}

// functions
void move_forward()
{
  flag_nav = 'F';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}

void adjust_left()
{
  flag_nav = 'L';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(200);
}

void adjust_right()
{
  flag_nav = 'R';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(200);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}

void stop_move()
{
  flag_nav = 'P';
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);
}
