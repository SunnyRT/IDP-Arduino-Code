// SET the value below for testing
int box_intend = 1;
/*
   box_intend = 1 --> deliver in green box
   box_intend = 3 --> deliver in red box
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT
int motor_speed = 200;
int duration_steer = 1400; // require testing to determine value
int duration_start_forward = 1400;
Servo servo_claw;



// Pins Set-up:
//Digital:
const int button_pn = 5;
const int l0_pn = 2; //left
const int l1_pn = 3; //right
const int l2_pn = 4; //far right (for juntion counting)
const int servo1_pn = 9;
const int ledA_pn = 10;


// Sensor Values
int l0, l1, l2, hall;


// button debounce variables
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

/*************
   flags
**************/
bool flag_onoff = false; // push button: robot on or off
bool flag_ledA;
char flag_nav = 'P';
bool flag_blk = true; // assume block has been collected in this test!!!
bool flag_delivered = false;
int box_pass = 0;
bool flag_box_register;


unsigned long return_home_time = 0;
int return_home_duration = 5100;




void setup() {
  Serial.begin(9600);
  // set pins as inputs
  pinMode(button_pn, INPUT);
  pinMode(l0_pn, INPUT);
  pinMode(l1_pn, INPUT);
  pinMode(l2_pn, INPUT);
  pinMode(servo1_pn, OUTPUT);
  pinMode(ledA_pn, OUTPUT);
  AFMS.begin(); // Connect to the motor controller
  servo_claw.attach(servo1_pn);  // connect to servo
  servo_claw.write(0);
}






void loop() {

  /*read & print line sensors*/
  Serial.print("Line Sensors: ");
  l0 = digitalRead(l0_pn);
  Serial.print(l0);
  l1 = digitalRead(l1_pn);
  Serial.print(l1);
  l2 = digitalRead(l2_pn);
  Serial.println(l2);



  /** On-off Push Button:*/
  // if the time elapsed is greater than time for debounce:
  if (millis() - lastTimeButtonStateChanged > debounceDuration) {
    // read button state
    byte buttonState = digitalRead(button_pn);
    // if button state has changed
    if (buttonState != lastButtonState) {
      // update time
      lastTimeButtonStateChanged = millis();
      // update lastButton state
      lastButtonState = buttonState;
      // while being pressed
      if (buttonState == LOW)
      {
        // do an action, for example print on Serial
        Serial.println("Button released");
        flag_onoff = !flag_onoff;
      }
    }
  }
  Serial.print("ONOFF: ");
  Serial.println(flag_onoff);
  if (flag_onoff == false && flag_ledA == true ) {
    digitalWrite(ledA_pn, LOW);
  }
  else if (flag_onoff == true && flag_ledA == false) {
    digitalWrite(ledA_pn, HIGH);
  }






  /*********************************************************************************

   ********************************************************************************/
  if (flag_onoff == true) {
    Serial.print("box_intend: ");
    Serial.println(box_intend);
    Serial.print("box_pass: ");
    Serial.println(box_pass);

    Serial.print("flag_box_register: ");
    Serial.println(flag_box_register);


    if (flag_delivered == false) //blk has yet to be delivered
    {
      if (l2 == LOW) //move back towards the green box
      {
        flag_box_register = false;
        line_follow();
      }
      else if (l2 == HIGH && l1 == HIGH )// junction detected! --> blk delivery
      {
        if (flag_box_register == false) {
          // if junction for box has not been registered
          flag_box_register = true; // at start of junction raise junction flag
          // increment the number of box-junctions passed
          box_pass += 1;
          Serial.print("flag_box_register (in if): ");
          Serial.println(flag_box_register);

        }
        if (box_pass == box_intend) //time to deliver the blk!!
        {
          Serial.println("junction detected!");
          blk_delivery();
          blk_retriet();
          return_home_time = millis();
        }
        else if (box_pass < box_intend) //not yet reached the intended box to deliver the blk.
        {
          move_forward();
        }
        else {
          Serial.println("Error: the robot missed the box to deliver!");
        }
      }
    }
    else //blk has been delivered, return home
    {
      if ((millis() - return_home_time) < return_home_duration) {
        line_follow();
      }
      else // junction detected! --> return home
      {
        Serial.println("junction detected!");
        return_home();
      }
    }
  }
  else if (flag_onoff == false && flag_nav == 'P')
  {
    stop_move();
  }
}









/*========= FUNCTIONS ===================*/
void line_follow() {
  if (l2 == LOW) {

    // 000 & it's not already moving forward
    if ((l0 == LOW && l1 == LOW) && flag_nav != 'F') {
      move_forward();
      Serial.println("Move Forward");
    }

    // 100 & not already moving left
    else if ((l0 == HIGH && l1 == LOW) && flag_nav != 'L') {
      adjust_left();
    }

    // 010 & not already moving right
    else if ((l0 == LOW && l1 == HIGH) && flag_nav != 'R') {
      adjust_right();
    }
  }

  // xx1 or 11x(junction detected)
  else if (l2 == HIGH || (l0 == HIGH && l1 == HIGH)) {
    // handle junction depending on flags
    Serial.println("Junction Detected");
  }

  // catch all other cases: stop & report error
  else {
    stop_move();
    Serial.print("Line Sensor Error: ");
    Serial.print(l0);
    Serial.print(l1);
    Serial.println(l2);
  }
}


/*********************************************************************************
   Functions related to navigation
 *********************************************************************************/

void stop_move() {
  flag_nav = 'P';
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);
}


void move_forward()
{
  flag_nav = 'F';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}

void move_backward() {
  flag_nav = 'B';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);

}

void adjust_left()
{
  flag_nav = 'L';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(100);
}

void adjust_right()
{
  flag_nav = 'R';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(100);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}

void turn_90left()
{
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer - 100);
}

void turn_90right()
{
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer + 50);
}

void turn_180() {
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer * 2); // How long it will turn for!

}



/*********************************************************************************
   Functions related to blk
 *********************************************************************************/

void blk_delivery()
{
  stop_move();
  delay(1000);
  turn_90right();
  stop_move();
  delay(1000);
  move_forward();
  delay(duration_start_forward);
  stop_move();
  delay(1000);
  servo_claw.write(0);
  delay(1000);
  flag_delivered = true;
}

void blk_retriet()
{
  move_backward();
  delay(duration_start_forward - 100);
  stop_move();
  delay(1000);
  if (box_intend == 1) {
    turn_90left();
  }
  else if (box_intend == 3) {
    turn_90right();
  }
  stop_move();
  delay(1000);
  move_forward();
  delay(200);
}

void return_home()
{
  stop_move();
  delay(1000);
  if (box_intend == 1) {
    turn_90right();
  }
  else if (box_intend == 3) {
    turn_90left();
  }
  stop_move();
  delay(1000);
  move_forward();
  delay(2.5 * duration_start_forward);
  stop_move();
  delay(1000);
  flag_onoff = false; //power off
  digitalWrite(ledA_pn, LOW);
}
