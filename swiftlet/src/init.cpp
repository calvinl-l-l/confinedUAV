#include <iostream>
#include "AP_controller.h"
#include "AP_interface.h"
#include "scheduler.h"

using namespace std;

int main()
{

    char* PH_PORT = "/dev/ttyUSB0";
    int PH_BAUD = 921600;
    Serial_Port pix_sp(PH_PORT, PH_BAUD);
    AP_interface quad(&pix_sp);
    pix_sp.start();

    Hokuyo_lidar lidar;
    position_controller fc;

    quad.read_msg();
    quad.set_startup_time();
    lidar.set_startup_time();

    start_scheduler(quad, lidar, fc);

    while (1)
    {
        delay(10000);
    }

    cout << "Hello world! This is the end!" << endl;
    return 0;
}
