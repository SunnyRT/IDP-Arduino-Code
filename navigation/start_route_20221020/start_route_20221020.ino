#include "functions.cpp";
#include "functions.h";

const int push_pn=2; //ideally not use pin 1 (move to 5?)
const int l0_pn=3; //left
const int l1_pn=4; //right
const int l2_pn=5; //far right (for juntion counting)




// Variables will change:
int button_state;            // the current reading from the input pin
int last_button_state = LOW; // the previous reading from the input pin
bool flag_started = false;
bool flag_onoff = false;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers

void setup()
{
    pinMode(push_pn, INPUT);

    // set initial state to off
    flag_onoff = false;
}

void loop()
{

    sensor_read(); //--> read push button and store the value into "push"

    /**********************************************************************************/
    /* push button debounce function */
    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (push != last_button_state)
    {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        // if the button state has changed:
        if (push != button_state)
        {
            button_state = push;

            // only toggle the LED if the new button state is HIGH
            if (button_state == HIGH)
            {
                flag_onoff != flag_onoff;
            }
        }
    }

    // save the reading. Next time through the loop, it'll be the lastbutton_state:
    last_button_state = push;

    /**********************************************************************************/

    if (flag_onoff == false && flag_nav != "P")
    {
        stop_move();
    }
    else if (flag_onoff == true && flag_started == false;)
    {
        start_route();
    }
    else
    {
        line_follow();
    }
}



/***********************************************************************/
// define functions
void start_route()
{
    flag_line = "off";
    if (l0 == HIGH && l1 == HIGH && flag_nav != "F")
    {
        move_forward();
    }
    else if (l0 == LOW && l1 == LOW)
    {
        turn_90right();
        flag_started = true; //exit start_route
    }
}

void sensor_read(){
  // light/line sensors:
    // ldr = analogRead(ldr_pn);
    // hall = analogRead(hall_pn);
    // ir1 = analogRead(ir1_pn);
    // ir2 = analogRead(ir2_pn);
    push = digitalRead(push_pn);
    l0 = digitalRead(l0_pn);
    l1 = digitalRead(l1_pn);
    l2 = digitalRead(l2_pn);
    l3 = digitalRead(l3_pn);
    // us1_distance = us_measure(us1T_pn, us1E_pn);
    // us2_distance = us_measure(us2T_pn, us2E_pn); 
};
