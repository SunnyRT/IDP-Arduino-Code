// header guard
#ifndef sensor_H
#define sensor_H
#include "start_up.h"

void increment_loop_count(){
  if (loop_count == 5) {
    loop_count = 0;
  }
  else {
    loop_count += 1;
  }
}

void register_junction(){
    // if junction for box has not yet been registered
    // flag_box_register is a variable used to ensure each junction is only counted once
    if (flag_box_register == false) 
    {// at start of junction raise junction flag
        flag_box_register = true; 
        box_pass += 1; // increment the number of box-junctions passed
        Serial.print(box_pass);
    }
}


void line_sensors_read(){
    // read & print line sensors
    // this was used regularly for debugging issues with navigation & steering
    Serial.print("Line Sensors: ");
    l0 = digitalRead(l0_pn);
    Serial.print(l0);
    l1 = digitalRead(l1_pn);
    Serial.print(l1);
    l2 = digitalRead(l2_pn);
    Serial.println(l2);    
}


void push_button_state(){
    /** On-off Push Button:*/
    // if the time elapsed is greater than time for debounce:
    currentTime = millis(); // read current time
    // if time difference is greater than debounce time
    if (currentTime - lastTimeButtonStateChanged > debounceDuration) {
        // read button state
        byte buttonState = digitalRead(button_pn);
        // if button state is different since the last
        if (buttonState != lastButtonState) {
        // update the time for last-changed button
        lastTimeButtonStateChanged = currentTime;
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
}

void ledAswitch(){
    // toggle amber LED if on (about to start moving)
     if (flag_onoff == false && flag_ledA == true ) {// if robot is off & ledA is still on, 
    digitalWrite(ledA_pn, LOW);// turn it off
    }
    else if (flag_onoff == true && flag_ledA == false) {
        digitalWrite(ledA_pn, HIGH);
    }
}


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

void tunnel1(){
    while ((currentTime - started_time) > 14500 && (currentTime - started_time) < 19500) {
        us2_measure();
        Serial.print("us2: ");
        Serial.println(us2_distance);
        tunnel_P_control(distance_tunnel1);
        Serial.println("moving in tunnel...");
        currentTime = millis();
    }
    us2_distance = 200; // to re-initialize the value of us2_distance
}

#endif