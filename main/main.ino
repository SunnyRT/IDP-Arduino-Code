// include header files:
#include functions.h

// Pins Set-up:
//Analog:
const int ldr_pn = 0;
const int hall_pn = 1;
const int ir1_pn = 4; // short 
const int ir2_pn = 5;
//Digital:

const int push_pn=1; //ideally not use pin 1 (move to 5?)
const int l0_pn=2; //left
const int l1_pn=3; //right
const int l2_pn=4; //far right (for juntion counting)
const int l3_pn=5;
const int us1E_pn=6;
const int us1T_pn=7;
const int ledG_pn=8;
const int ledA_pn=9;
const int ledR_pn=10;
const int servo1_pn=11;
const int us2E_pn=12;
const int us2T_pn=13;


void setup(){
  // set pins as inputs
  pinMode(ldr_pn, INPUT);
  pinMode(hall_pn, INPUT);
  pinMode(ir1_pn, INPUT); 
  pinMode(ir2_pn, INPUT);
  pinMode(l0_pn, INPUT);
  pinMode(l1_pn, INPUT);
  pinMode(l2_pn, INPUT);
  pinMode(l3_pn, INPUT);
  pinMode(ledR_pn, OUTPUT);
  pinMode(ledA_pn, OUTPUT);
  pinMode(ledG_pn, OUTPUT);
  pinMode(us1E_pn, INPUT);


}


