#include <iostream>
#include <wiringPi.h>
#include "src/pixhawk_interface.h"
#include "src/scheduler.h"
#include "src/utility.h"
#include "src/mavlink/serial_port.h"



using namespace std;


int main()
{
   // wiringPiSetup();

    // Pixhawk setup
    char* PIXHAWK_PORT = "/dev/ttyACM0";
    int PIXHAWK_BAUD = 115200;
    Serial_Port pix_sp(PIXHAWK_PORT, PIXHAWK_BAUD);
    Pixhawk_Interface quad(&pix_sp);
    pix_sp.start();
    quad.start();

    Hokuyo_lidar lidar;
    position_controller fc;

    // start all threads
    start_scheduler(quad, lidar, fc);

    while (1)
    {
        //cout << "herelodso" << endl;
        //cout << 'i' << quad.pitch << endl;
        delay(1000);
    }

    return 0;
}

/*
TO DO:
- need a start thread flag


lidar_1
PID values => kp: 0.15 ki: 0.01 kd: 0.085
Print out format:
yc, zc, alt, CH1, CH3, is_manual?, roll, yaw, area, system time, RF, pitch, CH2, CH4


scan_1
n_pt angle range time
*/
