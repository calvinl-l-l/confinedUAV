#include <iostream>
#include "lib/Scheduler/Scheduler.h"

using namespace std;

int main()
{

    char* PH_PORT = "/dev/ttyUSB0";
    int PH_BAUD = 921600;

    Hokuyo_lidar lidar;



    while (1)
    {
        delay(10000);
    }

    cout << "Hello world! This is the end!" << endl;
    return 0;
}
