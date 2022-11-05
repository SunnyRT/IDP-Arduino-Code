#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
//#include "functions.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT

/*defining pins and variables*/

#define button_pn 5 // switch
#define l0_pn 2      // left
#define l1_pn 3      // right
#define l2_pn 4      // far right
const int ledG_pn=8;
const int ledA_pn=10;
const int ledR_pn=9;

// Variables will change:
bool flag_started = false;
bool flag_onoff = false;
char flag_nav = 'P';
int l0, l1, l2;

int motor_speed = 200;
int duration_steer = 2000;

// button variable
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;


//ledA_flash variables
int ledAState = LOW;  // ledState used to set the LED
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 250;  // 2hz --> 0.5 second interval at which to blink (milliseconds)

void setup()
{
    pinMode(button_pn, INPUT);
    pinMode(l0_pn, INPUT); // light sensor on the left
    pinMode(l1_pn, INPUT); // light sensor on the right
    pinMode(l2_pn, INPUT); // light sensor on the far right
    pinMode(ledA_pn, OUTPUT);
    Serial.begin(9600);
    AFMS.begin(); // Connect to the controller
}

void loop()
{
    // read and store light sensor values

    ledA_flash(); //too dim!!!!
    
    
    l0 = digitalRead(l0_pn);
    l1 = digitalRead(l1_pn);
    l2 = digitalRead(l2_pn);

    // print values of the light sensors to the serial monitor
    Serial.print("Sensor Readings: ");
    Serial.print(l0);
    Serial.print(l1);
    Serial.println(l2);

    Serial.print("Flag States: ");
    Serial.println(flag_nav);
    Serial.println("Flag Started: ");
    Serial.println(flag_started);

    // button debounce
    if (millis() - lastTimeButtonStateChanged > debounceDuration)
    {
        byte buttonState = digitalRead(button_pn);
        if (buttonState != lastButtonState)
        {
            lastTimeButtonStateChanged = millis();
            lastButtonState = buttonState;
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

    if (flag_onoff == false && flag_nav != 'P')
    {
        stop_move();
    }

    else if (flag_onoff == true && flag_started == false)
    {
        start_route();

    }
    else if (flag_onoff == true && flag_started == true)
    {
        line_follow();
    }
}

/***********************************************************************/
// define functions of start_route
void start_route()
{
    if (l0 == LOW && l1 == LOW && l2 == LOW && flag_nav != 'F')
    {
        move_forward();
    }
    else if ((l0 == HIGH && l1 == HIGH) || l2 == HIGH)
    {
        move_forward();
        delay(800);
        turn_90left();
        flag_started = true; // exit start_route
    }
}



void line_follow() {
if (l2 == HIGH && flag_nav != 'F')
  {
    //ignore junctions, just go forward;
    move_forward();
  }

  // if no junctions are detected
else if (l2 == LOW)
  {
    // neither l0 or l1 detects the line
    if ((l0 == LOW && l1 == LOW) && flag_nav != 'F')
    {
      // forward
      move_forward();
      Serial.println("MOVE FORWARD!");
    }

    // l0 detects the line
  else if ((l0 == HIGH && l1 == LOW) && flag_nav != 'L')
    {
      // adjust left slightly
      adjust_left();
    }

    // l1 detects the line
  else if ((l0 == LOW && l1 == HIGH) && flag_nav != 'R')
    {
      // adjust right slightly
      adjust_right();
    }

    // l0 and l1 both detect the line
    else if ((l0 == HIGH && l1 == HIGH) && flag_nav != 'P')
    {
      // forward
      stop_move();
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

void stop_move()
{
    flag_nav = 'P';
    Rwheel->run(RELEASE);
    Rwheel->setSpeed(0);
    Lwheel->run(RELEASE);
    Lwheel->setSpeed(0);
}

void turn_90left()
{
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(motor_speed);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(50);
    delay(duration_steer);
}



// amber LED flashes at 2Hz while robot is running
void ledA_flash()
{
  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // toggle LED
    if (ledAState == LOW)
    {
      ledAState = HIGH;
    }
    else
    {
      ledAState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledA_pn, ledAState);
  }
}
