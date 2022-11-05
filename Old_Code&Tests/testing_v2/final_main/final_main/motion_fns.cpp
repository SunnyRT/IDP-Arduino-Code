#include "motion_fns.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>



/*********************************************************************************
   Functions related to navigation
 *********************************************************************************/





void blk_find()
{
  digitalWrite(ledA_pn, LOW);
        stop_move();
        delay(1000);

        //get closer to the blk for magnet detection
        move_forward();
        delay(500);
        stop_move();
        delay(1000);
        blk_magnet();

        //move backward a little bit for blk collection
        move_backward();
        delay(300);
        stop_move();
        delay(1000);
        blk_LEDindication();
        blk_collect();
        digitalWrite(ledA_pn, HIGH);
}


void line_follow() {
  if (l2 == LOW) {

    // 000 & it's not already moving forward
    if ((l0 == LOW && l1 == LOW) && flag_nav != 'F') {
      move_forward();
      //      Serial.println("Move Forward");
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
    //    Serial.println("Junction Detected");
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

void start_route() {
  // if still in the box (so no line detected), move forward until edge of box detected
  if (l0 == LOW && l1 == LOW && l2 == LOW && flag_nav != 'F')
  {
    move_forward();
  }
  // edge of box detected
  // maybe change this condition slightly, maybe if all are high?
  else if ((l0 == HIGH && l1 == HIGH) || l2 == HIGH)
  {
    move_forward();
    delay(duration_start_forward - 100);
    stop_move();
    delay(1000);
    turn_90left();
    flag_started = true; // exit start_route
  }
}


/*********************************************************************************
   Functions related to blk
 *********************************************************************************/
void blk_magnet()
{
  for (int i = 0; i < 10; i++) //obtain 10 readings from hall effect sensor
  {
    delay(100);
    hall = digitalRead(hall_pn);
    Serial.println(hall);
    if (hall == LOW) //magnet is detected when reading is  0
    {
      box_intend = 3;
    }
  }
  Serial.println("finish detecting!");
  Serial.print("box_intend: ");
  Serial.println(box_intend);
}


void blk_LEDindication()
{
  if (box_intend == 3) //magnet present
  {
    analogWrite(ledR_pn, 255);
    delay(5000);
    analogWrite(ledR_pn, 0);
  }
  else //no magnet present
  {
    analogWrite(ledG_pn, 255);
    delay(5000);
    analogWrite(ledG_pn, 0);
  }
}

void blk_collect()
{
  // rotate claw to trap the blk
  servo_claw.write(45);
  delay(1000);
  turn_180();
  Serial.println("finish turning!");
  stop_move();
  delay(1000);
  flag_blk = true;
}


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
  delay(1200);
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
    Rwheel->run(BACKWARD);
    Rwheel->setSpeed(motor_speed);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(motor_speed);
    delay(duration_steer -100);
  }
  else if (box_intend == 3) {
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(motor_speed);
    Lwheel->run(BACKWARD);
    Lwheel->setSpeed(motor_speed);
    delay(duration_steer - 125);
  }
  stop_move();
  delay(1000);
  us1_distance = 200;
  while(us1_distance > 8){
    us1_measure();
    move_forward();
  }
  stop_move();
  analogWrite(ledG_pn, 255);
  analogWrite(ledR_pn, 255);
  delay(5000);
  flag_onoff = false; //power off
  digitalWrite(ledA_pn, LOW);
  analogWrite(ledG_pn, 0);
  analogWrite(ledR_pn, 0);
}




/*********************************************************************************
   Functions related to ultrasonic sensor readings
 *********************************************************************************/

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


void tunnel_PID_control(float distance_wall)
{
  float error = distance_wall - us2_distance;

  P = error;
  float motorspeed = P * Kp;

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