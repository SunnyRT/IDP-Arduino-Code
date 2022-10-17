// header guard
#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>



// global
void sensor_read();
void line_follow();
void stop_move();
void move_foward();
void move_backward();
void adjust_left();
void adjust_right();
void turn_90left();
void turn_90right();

// side == 0;
void start_route();

// side == 1;
void ramp_up();
void ramp_down();

// side == 2;
void blk_fn();

void blk_approach();
void blk_magnet_identify();
void blk_magnet_indicate();
void blk_collect();
void blk_retriet();

// side == 3;
void tunnel();

// side == 4;
void box_find();

void box_delivery();

#endif