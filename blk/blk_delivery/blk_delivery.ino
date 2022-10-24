// variables
int l0, l1, l2

// flags
extern int side;
extern int count;
extern int flag_onoff;
extern bool flag_blk;
extern bool flag_magnet;
bool flag_box_reg = false;

extern int box_intend;
extern int box_pass;

void loop()
{
    // conditions: the block has been collected with the robot, the robot is on the delivery side.
    while (flag_blk == true && side == 1) // local loop within this section
    { 
        sensor_read();
        line_follow();
        if (l2 == LOW && flag_box_reg == false)
        {
            box_pass += 1;
            flag_box_reg = true;
        }
        else if (l2 == HIGH)
        {
            flag_box_reg = false; // ready for registration of the next box
        }

        if (box_pass == box_intend)
        {
            blk_delivery();
        }
    }

    if (flag_blk == false && side == 1)
    {
        if (box_intend == 1)
        {
            move_forward(); // towards the middle box
        }

        else if (box_intend == 3)
        {
            move_backward(); // towards the middle box
        }

        delay(2000); // timing requires calibration

        // turn towards the box
        turn_90right(); // functions need to be updated in the start_rout_20221020
        delay(1000);    // timing requires calibration

        // move into the box
        move_forward();
        delay(1000); // timing requires calibration

        stop_move(); 
        flag_onoff == false;
        Serial.print("MISSION COMPLETE!!!") // finish off
    }
}

void line_follow() {}
void sensor_read() {}

void blk_delivery()
{
    // turn towards the box
    turn_90right(); // functions need to be updated in the start_rout_20221020
    delay(1000);    // timing requires calibration

    // move into the box
    move_forward();
    delay(1000); // timing requires calibration

    stop_move();
    // lift up the claw by the servo to release the blk
    servo_claw.write(0); // the value here requires alibration
    delay(1000);

    flag_blk = false; // the blk has been delivered

    move_backward();
    delay(1000);
    turn_90left();
    delay(1000);  // return to the main route with lines
}