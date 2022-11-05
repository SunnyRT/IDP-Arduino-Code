#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT
int motor_speed = 250;
int duration_steer = 2000; // require testing to determine value
Servo servo_claw; 

// Pins Set-up:
//Analog:
const int ledG_pn=A2; // now analog
const int ledR_pn=A3; // now analog


//Digital:
const int button_pn=5; 
const int l0_pn=2; //left
const int l1_pn=3; //right
const int l2_pn=4; //far right (for juntion counting)

//us1: on the front
const int us1E_pn=6; // yellow wire
const int us1T_pn=7; // green wire
const int hall_pn = 8;
const int servo1_pn=9; // in-built port for servos (side)
const int ledA_pn=10;


//us2: on the left
const int us2E_pn=12;
const int us2T_pn=13;

// Sensor Values
int l0, l1, l2, hall;
float us1_distance, us2_distance;

// button debounce variables
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

//ledA_flash variables
int ledAState = LOW;  // ledState used to set the LED
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 250;  // 2hz --> 0.5 second interval at which to blink (milliseconds)

// PID tunnel navigation variables
float lastError = 0;
float Kp = 0.07; // related to the proportional control term;
// change the value by trial-and-error (ex: 0.07).
float Ki = 0.0008; // related to the integral control term;
// change the value by trial-and-error (ex: 0.0008).
float Kd = 0.6; // related to the derivative control term;
// change the value by trial-and-error (ex: 0.6).
float P;
float I;
float D;
float distance_tunnel = 7;

const float maxspeedR = 150;
const float maxspeedL = 150;
const float basespeedR = 100;
const float basespeedL = 100;


/*************
 * flags
**************/
int side; // 0,1,2,3,4,5
int count = 0;
bool flag_onoff = false; // push button: robot on or off
bool flag_started;       // has robot completed initial start?
bool flag_line;
int flag_side; 
char flag_nav; 
  //"P": stop
  //"L": adjust left
  //"R": adjust right
  //"B": backwards
  //"F": forwards
bool flag_blk; // block collected
bool flag_magnet;
bool flag_tunnel; // true if in tunnel, false if not
int box_intend;

/*************
 * declare functions
**************/
void stop_move();
void start_route();
void move_forward();
void adjust_left();
void adjust_right();


void setup(){
  Serial.begin(9600);
  // set pins as inputs
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
}
/*************
 * MAIN LOOP =======================================
**************/
void loop(){
    /*read & print line sensors*/ 
    Serial.print("Line Sensors: ");
    l0 = digitalRead(l0_pn);
    Serial.print(l0);
    l1 = digitalRead(l1_pn);
    Serial.print(l1);
    l2 = digitalRead(l2_pn);
    Serial.println(l2);

    // flags
    Serial.print("Flag_nav: ");
    Serial.println(flag_nav);
    Serial.print("Flag Started: ");
    Serial.println(flag_started);

    /** On-off Push Button:*/
    // if the time elapsed is greater than time for debounce:
    if (millis() - lastTimeButtonStateChanged > debounceDuration){
        // read button state
        byte buttonState = digitalRead(button_pn);
        // if button state has changed
        if (buttonState != lastButtonState){
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
    
    /** Navigation */
    // if 'Off' and not already stopped, stop:
    if(flag_onoff==false && flag_nav != 'P'){
        stop_move();
    }
    // 'ON' and not running the start function:
    if(flag_onoff==true && flag_started==false){
        start_route();
    }
    // if On and the start fn has been completed:
    if(flag_onoff==true && flag_started==true){

        /** Block Detection*/
        if(us1_distance < 10){
            
            // approach slowly until us1_distance = 2cm 
            stop_move();
            Serial.println("Block Detected");
            delay(1000);
            Serial.println("Start detecting if magnet");
            /* blk function */
            if (hall == 0){
                // magnet
                Serial.println("Magnet");
                flag_magnet = 1;
                box_intend = 3; // blk is to be delivered in the red box
                // RED LED for 5sec
                analogWrite(ledR_pn, 255);
                delay(5000);
                analogWrite(ledR_pn, 0);
            }
            else{
                // no magnet
                Serial.println("Not Magnet");
                flag_magnet = 0;
                box_intend = 1; // bl
                // Green LED for 5sec
                analogWrite(ledG_pn, 255);
                delay(5000);
                analogWrite(ledG_pn, 0);
            }

            /** claw  code */

            flag_blk = true;
        }

        /** No Block Detected:*/
        else {
            /* line-follow or side-sensor navigation ?*/
            // if (*RIGHT*) sensor (us2) < 6cm, entering tunnel
            if((us2_distance < 6)){
                flag_tunnel = true;
                /** PID tunnel navigation */
                int error = 5.0 - us2_distance; // 5.0 is the ideal distance from the tunnel wall
                P = error;
                I = I + error;
                D = error - lastError;
                lastError = error;
                int motorspeed = P * Kp + I * Ki + D * Kd; // calculate the correction
                // needed to be applied to the speed
                int motorspeedR = basespeedR + motorspeed;
                int motorspeedL = basespeedL - motorspeed;

                if (motorspeedR > maxspeedR){
                    motorspeedR = maxspeedR;
                }
                if (motorspeedL > maxspeedL){
                    motorspeedL = maxspeedL;
                }
                if (motorspeedR < 0){
                    motorspeedR = 0;
                }
                if (motorspeedL < 0){
                    motorspeedL = 0;
                }

                flag_nav = 'F';
                Rwheel->run(FORWARD);
                Rwheel->setSpeed(motorspeedR);
                Serial.print("speedR: ");
                Serial.println(motorspeedR);

                Lwheel->run(FORWARD);
                Lwheel->setSpeed(motorspeedL);
                Serial.print("speedL: ");
                Serial.println(motorspeedL);
            }

            else { /* line follow */

                // xx0 (if far-right sensor has no line)
                if(l2==LOW){

                    // 000 & it's not already moving forward
                    if((l0 == LOW && l1 == LOW) && flag_nav != 'F'){
                        move_forward();
                        Serial.println("Move Forward");
                    }

                    // 100 & not already moving left
                    else if((l0 == HIGH && l1 == LOW) && flag_nav != 'L'){
                        adjust_left();
                    }

                    // 010 & not already moving right
                    else if ((l0 == LOW && l1 == HIGH) && flag_nav != 'R'){
                        adjust_right();
                    }
                }

                // xx1 or 11x(junction detected)
                else if(l2==HIGH || (l0==HIGH && l1==HIGH)){
                    // handle junction depending on flags
                    Serial.println("Junction Detected");
                }

                // catch all other cases: stop & report error
                else{
                    stop_move();
                    Serial.print("Line Sensor Error: ");
                    Serial.print(l0);
                    Serial.print(l1);
                    Serial.println(l2);
                }
            } // close line follow
        } // close if nothing detected in front
    } // close 'On' 
 }// close loop


/*========= FUNCTIONS ===================*/
void start_route(){   
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
        delay(800);
        turn_90left();
        flag_started = true; // exit start_route
    }
}


void stop_move(){
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
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(50);
    delay(duration_steer);
}

void turn_180(){
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(motor_speed);
    Lwheel->run(BACKWARD);
    Lwheel->setSpeed(motor_speed);
    delay(1000); // How long it will turn for!

}