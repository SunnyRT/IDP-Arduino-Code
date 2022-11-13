// header guard
#ifndef block_H
#define block_H
#include "start_up.h"
#include "sensor_functions.h"

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

void blk_collect()
{
  // rotate claw to trap the blk
  trap_blk();
  delay(1000);
  turn_180();
  Serial.println("finish turning!");
  stop_move();
  delay(1000);
  flag_blk = true;
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
  release_blk();
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


void block_found(){
    digitalWrite(ledA_pn, LOW); // turn off amber LED as we are no longer moving
    stop_move();
    delay(1000);
    //get closer to the block to be close enough for magnet detection
    move_forward();
    delay(500);     // these timings were experimentally determined
    stop_move();
    delay(1000);
    blk_magnet();
    //move backward a little bit for block collection
    move_backward();
    delay(300);
    stop_move();
    delay(1000);
    blk_LEDindication();    // light up the correct LED
    blk_collect();          // servo used to lower collection arm 
    digitalWrite(ledA_pn, HIGH); // light up amber LED again
}

#endif
