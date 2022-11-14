#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include "start_up.h" // set up variables, declare functions
#include "sensor_functions.h" // define functions relating to sensors
#include "block_functions.h"  // defin functions relating to interactions with the block

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIGHT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT
Servo servo_claw;

void setup(){
    Serial.begin(9600);
  // set pins as inputs: see start_up.h file for pin numbers
  pinMode(button_pn, INPUT);
  pinMode(hall_pn, INPUT);
  pinMode(l0_pn, INPUT);
  pinMode(l1_pn, INPUT);
  pinMode(l2_pn, INPUT);
  pinMode(us1E_pn, INPUT);
  pinMode(us1T_pn, OUTPUT);
  pinMode(ledG_pn, OUTPUT);
  pinMode(ledA_pn, OUTPUT);
  pinMode(ledR_pn, OUTPUT);
  pinMode(servo1_pn, OUTPUT);
  pinMode(us2E_pn, INPUT);
  pinMode(us2T_pn, OUTPUT);
  AFMS.begin(); // Connect to the motor controller
  servo_claw.attach(servo1_pn);  // connect to servo
  servo_claw.write(0);
}

void loop(){
    increment_loop_count(); // used so that the ultrasonic is only measured every 5 loops
    line_sensors_read(); // read values from all line sensors
    push_button_state(); // including debouncing
    ledAswitch(); // turns amber LED on

    // if 'Off' and not already stopped, stop:
    if (flag_onoff == false && flag_nav != 'P') {
        stop_move();
    }
    // 'ON' and not running the start function:
    else if (flag_onoff == true && flag_started == false) {
        start_route();
        started_time = millis(); // allows us to measure time elapsed since robot started
        // so it can return within 5 minute time limit
    }
    // if On and the start has been completed:
    else if (flag_onoff == true && flag_started == true){
        /** if no block is collected*/
        if (flag_blk == false)
        { 
            /*  if time elapsed since starting time > 14500ms & <19500ms, assume robot is in tunnel
            We are very aware that this is not a rigorous solution, but it was a short-term
            fix on the day of competition, and we want this documentation to accurately reflect
            what code was run to yield the competition result that was marked.
            A quick fix was required because we did not have a distance sensor on the right hand side
            side of the robot to detect the start of the tunnel. This was because we had originally planned 
            only to go around the course in one direction only, and abandoned the tunnel too late for 
            our mechanical & electrical team to add another sensor in.
            */
            tunnel1();

            if (loop_count == 0) {us1_measure();}  // measure front distance sensor
            // if no block is found (front distance value is greater than 10)
            if (us1_distance > 10.0) {
                line_follow();
                Serial.println("line");
            }
            // else - the block is found
            else {
                block_found();
            }
        }
      /** Return Jouney: */
      else // blk has been collected, return route (flag_blk == true)
      {
        if (flag_delivered == false) //blk has yet to be delivered
        {
            /** before tunnel*/
            if (flag_tunnel == 0) // before the tunnel --> try to identify when tunnel is reached (us2_distance > 12.0)
            {
                if (loop_count == 0) {us2_measure();} // measure left distance sensor every 5 loops
                if (us2_distance > 12.0) // not yet reached the tunnel --> line follow
                {
                    line_follow();
                }
                else {
                    flag_tunnel = 1; // us2_distance < 12.0 --> tunnel has been reached
                }
            }
            /** within tunnel*/
            else if (flag_tunnel == 1) // within the tunnel
            {   
                us2_measure();  // us2 readings every loop
                if (us2_distance < 12.0) // moving within the tunnel
                {
                    tunnel_P_control(distance_tunnel2);
                }
                else {
                    flag_tunnel = 2;  // us2_distance > 12.0 --> tunnel has been passed
                }
            }
            /** after tunnel */
            else if (flag_tunnel == 2)
            {   // continue to move towards the box to deliver block

                // if no junction is detected (l2 is the far right line sensor)
                if (l2 == LOW) 
                {
                    flag_box_register = false;
                    line_follow();
                }

                else if (l2 == HIGH && l1 == HIGH )// junction detected! --> blk delivery
                { register_junction();
                    
                    if (box_pass == box_intend) //time to deliver the blk!!
                    {
                    blk_delivery();
                    blk_retriet();
                    return_home_time = millis();
                    }
                    // else if not yet reached the intended box to deliver the blk:
                    else if (box_pass < box_intend) 
                    {
                    line_follow();
                    }
                    // if we've missed the box intend, print error (for debugging)
                    else {
                    Serial.println("Error: the robot missed the box to deliver!");
                    }
                }
            } // after the tunnel
        } // blk has not been delivered (flag_delivered == false)

        else // (flag_delivered == true) blk has been delivered, return home
        {   //move back towards the middle
            // timing-controlled return to central box
            if ((currentTime - return_home_time) < return_home_duration) 
            {
            line_follow();
            }
            else // --> return home
            {
            return_home();
            }
        }

    }
}
}

/** Functions: */
// these are all functions relating directly to the motion of the robot

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
  // if edge of box detected:
  else if ((l0 == HIGH && l1 == HIGH) || l2 == HIGH)
  {
    // exit box and turn left onto the line
    move_forward();
    delay(duration_start_forward - 100);
    stop_move();
    delay(1000);
    turn_90left();
    flag_started = true; // exit start_route
  }
}


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
  Lwheel->setSpeed(motor_speed + 6);
}

void move_backward() {
  flag_nav = 'B';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed + 6);
}

void adjust_left()
{
  flag_nav = 'L';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(80);
}

void adjust_right()
{
  flag_nav = 'R';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(80);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}

void turn_90left()
{
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer - 75);
}

void turn_90right()
{
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer -25);
}

void turn_180() {
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
  delay(2775); // How long it will turn for, experimentally determined
  Serial.println("turning 180...");
}


void trap_blk(){
  servo_claw.write(45);
}

void release_blk(){
  servo_claw.write(0);
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


void tunnel_P_control(float distance_wall)
{   float Kp = 10; // related to the proportional control term;
    float P;

    const float maxspeedR = 200.0;
    const float maxspeedL = 200.0;
    const float basespeedR = 150.0;
    const float basespeedL = 150.0;

    // proportional control for differential steering
    float error = distance_wall - us2_distance;
    float motorspeed = error * Kp;

    
    float motorspeedR = basespeedR - motorspeed;
    float motorspeedL = basespeedL + motorspeed;
    /* these if statements are to handle the cases when the desired 
    motorspeed calculated is greater than the max motorspeed, and the 
    same for if it is lower than 0 */
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
