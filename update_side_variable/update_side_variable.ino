int side;
bool flag_line;

float us1_avg, last_us1_avg, us2_avg;

void update_side_variable()
{

    // call the function in every loop

    // flag_line is not true only when the robot exits the main square route

    // in case of dramatic changes in right distance readings from us1
    //    --> turning at the corners
    //  assume us readings in unit cm!!
    if (flag_line == false || abs(last_us1_avg - us1_avg) > 0.5)
    {
        side = 0; // indicating turning state
    }
    else
    {
        if (47.5 <= us1_avg <= 48.5) // d_R1 = 48cm
        {
            side = 1; // indicating the starting side
        }
        else if (8.5 <= us1_avg <= 9.5) // d_R2 = 9cm
        {
            side = 2; // indicating the ramp side
        }
        else if (11.5 <= us1_avg <= 12.5) // d_R3 = 12cm
        {
            side = 3; // indicating the collection side
        }
        else if (7.5 <= us1_avg <= 8.5) // d_R4 = 8cm
        {
            side = 4; // indicating the side with the tunnel
        }
        else if (us1_avg <= 6.0 && us2_avg <= 6.0)
        {
            side = 5; // indicating within the tunnel
        }
        else {
            // report error
            Serial.println("SIDE UNIDENTIFIED!!!!")
        }
    }
}

