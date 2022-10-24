

// flags
extern int side;
extern int count;
extern bool flag_blk;
extern bool flag_magnet;

extern int box_intend;
extern int box_pass;

// sensor readings
extern float ir_avg;
extern float hall;

void loop()
{
    if (side == 3)
    {
        // first time & blk is not collected
        if (count == 0 & flag_blk == false)
        {
            while (ir_avg >= 5) // the threshold value here requires measurement & calibration
            {
                // approach the blk until it is 5cm away
                line_follow();
            }
            // the robot is 5cm away from the blk detected
            Serial.println("BLK found!!");
            stop_move();
            blk_magnet_identify();
            blk_collect();
        }
    }
}

void line_follow(); // done
void stop_move();   // done


void blk_magnet_identify()
{
    if (hall <= 5) // the threshold value here requires measurement & calibration
    {
        // magnet is detected in the blk
        flag_magnet = 1;
        box_intend = 3; // blk is to be delivered in the red box

        // light up the red LED for 5 sec
        digitalWrite(ledR_pn, HIGH);
        delay(5000);
    }
    else
    {
        // no magnet in the blk
        flag_magnet = 0;
        box_intend = 1; // blk is to be delivered in the green box

        // light up the green LED for 5 sec
        digitalWrite(ledG_pn, HIGH);
        delay(5000);
    }
}

void blk_collect()
{
    while (ir_avg >= 0) // the threshold value here requires measurement & calibration
    {
        // continue to approach the blk until it touches.
        line_follow();
    }

    // touch the blk i.e. distance = 0 from the blk
    stop_move();

    // move the claw by the servo to trap the blk
    servo_claw.write(180); // the value here requires alibration
    delay(1000);

    flag_blk = true; //the blk has been collected
}


void blk_approach();
void blk_retriet();
