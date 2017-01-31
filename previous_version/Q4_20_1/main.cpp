#include <iostream>
#include "src/MSP.h"
#include "src/scheduler.h"
#include "src/utility.h"
#include <wiringPi.h>


using namespace std;


int main()
{
   // wiringPiSetup();

    MSP quad("/dev/ttyUSB0", 115200);
    Hokuyo_lidar lidar;

    start_scheduler(quad, lidar);

    while (1)
    {


        delay(10);
    }

    return 0;
}

/*
lidar_1
PID values => kp: 0.15 ki: 0.01 kd: 0.085
Print out format:
yc, zc, alt, CH1, CH3, is_manual?, roll, yaw, area, system time, RF, pitch, CH2, CH4


scan_1
n_pt angle range time
*/
