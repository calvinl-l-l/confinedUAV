// Custom controller
#include "../custom_header/controller.h"

//------------------ controller class functions ------------------

void controller_init()
{

}



// Altitude controller
int alt_controller(lidar_data ldata, int manual_ctrl)
{
    int thr_out = 1000;
    static int takeoff = 1;

    if ((ldata.zc < 250+150)&&(ldata.zc >= 150))
    {
        //thr_out = thr_input;
        thr_out = 1500;
    }
    else
    {
        if (ldata.zc >= 250+150)
        {
            thr_out = 1395;
        }
        else if (ldata.zc < 150)
        {
            thr_out = 1605;
        }

    }

    if (takeoff)
    {
        //thr_out = auto_takeoff(ldata, &takeoff, manual_ctrl);
    }

    return thr_out;
}// end of alt controller



// horizontal position controller
int horPos_controller(lidar_data ldata, float kp, float ki, float kd, int reset)
{
// input:
// hpos, previous hpos, current angle, prevous angle
    float dt = 0.1; // dt = 100ms

// output:
    int rc_rollOUT = 1500;


// variables
    float err = ldata.yc;
    float derr = ldata.prev_yc; // d_term = current - prev
    static double err_sum = 0;

    err_sum += err;

    // -------------------------------------------- integral
    float i_term = ki * err_sum * 0.1;

    if (i_term > integral_ulimit)   i_term = integral_ulimit;
    else if (i_term < integral_llimit) i_term = integral_llimit;

    if (reset)  err_sum = 0;
    //--------------------------------------------- integral

    //--------------------------------------------- derivative

    //----------------------------------------------derivative

// computation
    rc_rollOUT = 1500 + roll_offset + kp * err + i_term + kd * (err - derr)/dt; //dt = 0.02s

    if (rc_rollOUT > rc_ulimit)         rc_rollOUT = rc_ulimit;
    else if (rc_rollOUT < rc_llimit)    rc_rollOUT = rc_llimit;

    //cout << "i_term " <<i_term<<" r "<<reset<<" ki "<<ki<<" err "<<err<<" err sum "<<err_sum<<endl;

    // damping the output
    //rc_rollOUT = control_output_damper(rc_rollOUT);


    return rc_rollOUT;
}// end of horPos_controller


int auto_takeoff(lidar_data ldata, int* takeoff, int reset)
{
    int thr_out = 0;

    static int timer = 0;

    if (reset)
    {
        timer = 0;
        *takeoff = 1;
    }

    if (ldata.zc < 0)
    {
        thr_out = 1605;

        if ((timer >= 1000)&&(ldata.alt < 250))  thr_out = 1700;

        if (ldata.alt >= 300)   thr_out = 1605;
    }
    else
    {
        *takeoff = 0;
        thr_out = 1500;
    }

    timer += 20;

    return thr_out;
}


/*
int control_output_damper(int rc_out)

   static int out = 1500;
   static int current_out;
   static float step;

   if (rc_out==current_out)
   {
        out += (int) step;
   }
   else
   {
       current_out = rc_out;

       step = (current_out - out)/5;

       out += (int) step;
   }

   return out;
}
*/
