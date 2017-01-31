#include <iostream>
#include "src/MSP.h"
#include "src/scheduler.h"
#include "src/utility.h"
//#include <wiringPi.h>


using namespace std;


int main()
{
    MSP quad("/dev/ttyUSB0", 115200);

    start_scheduler(quad);

    while (1)
    {

    }

    return 0;
}


/*
int main()
{

    MSP naze("/dev/ttyACM1", 115200);

    uint16_t data[8] = {1500,1500,1000,2000,0,0,0,0};
    naze.send_msg(MSP_SET_RAW_RC, data, 16, 'w');

    delay(500);

    naze.send_msg(MSP_SET_RAW_RC, data, 16, 'w');

    delay(500);


    naze.send_msg(MSP_SET_RAW_RC, data, 16, 'w');

    cout << "armed quad" << endl;

    while (1)
    {

        uint16_t dataa[8] = {1500,1500,1100,1500,0,0,0,0};

        naze.send_msg(MSP_SET_RAW_RC, dataa, 16, 'w');

        naze.send_msg(MSP_RC, dataa, 0, 'r');

        naze.read_msg();

        delay(50);

        //cout << "M1 " << naze.motor.m1 << " M2 " << naze.motor.m2 << endl;
        cout << "pitch " << naze.rcIn.ch1 << " roll " << naze.rcIn.ch2 << " yaw " << naze.rcIn.ch3 << endl;
    }

    return 0;
}
*/

