// //Analog:
// const int ldr_pn = A0;
// const int hall_pn = A1;
// const int ir1_pn = A4; // short range
// const int ir2_pn = A5; // long range
// //Digital:
// const int button_pn=2; 
// const int l0_pn=3; //left
// const int l1_pn=4; //right
// const int l2_pn=5; //far right (for juntion counting)
const int us1E_pn=6; // yellow wire
const int us1T_pn=7; // green wire
// const int ledG_pn=8;
// const int ledA_pn=9;
// const int ledR_pn=10;
// const int servo1_pn=11;
// const int us2E_pn=12;
// const int us2T_pn=13;
// int motor_speed = 250;

// // Sensor Values
// int ldr, l0, l1, l2, l3, ir1, ir2, hall, push;
float us1_distance, us2_distance;
// float ir1_avg, ir2_avg, us1_avg, us2_avg;

float us_measure(int trig_pin, int echo_pin)
{
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float duration_us = pulseIn(echo_pin, HIGH);
  // return distance (*v_sound /2)
  return duration_us * 0.017;
}

// void sensor_read()
// {
//   // light/line sensors:
//   ldr = analogRead(ldr_pn);
//   hall = analogRead(hall_pn);
//   ir1 = analogRead(ir1_pn);
//   ir2 = analogRead(ir2_pn);
//   l0 = digitalRead(l0_pn);
//   l1 = digitalRead(l1_pn);
//   l2 = digitalRead(l2_pn);
//   // l3 = digitalRead(l3_pn);
//   us1_distance = us_measure(us1T_pn, us1E_pn);
//   us2_distance = us_measure(us2T_pn, us2E_pn);

//   // // calculate averages for distance readings
//   // ir1_avg = moving_avg(ir1);
//   // ir2_avg = moving_avg(ir2);
//   // us1_avg = moving_avg(us1_distance);
//   // us2_avg = moving_avg(us2_distance);

// };

void setup(){
  Serial.begin(9600);
  // pinMode(ir1_pn, INPUT); 
  pinMode(us1E_pn, INPUT);
  pinMode(us1T_pn, OUTPUT);

}

void loop(){
  // Serial.println("Test");
  us1_distance = us_measure(us1T_pn, us1E_pn);
  // us2_distance = us_measure(us2T_pn, us2E_pn);
  Serial.print("US1: ");
  Serial.println(us1_distance);
}

