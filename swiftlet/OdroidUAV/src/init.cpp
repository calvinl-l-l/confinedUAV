#include <iostream>
#include "../lib/Scheduler/Scheduler.h"
#include "../lib/cserial/cSerial.h"
#include <wiringPi.h>
#include "lidar.h"

using namespace std;

int main()
{

    char* PH2_PORT = "/dev/ttyUSB0";
    int PH2_BAUD = 115200;

    //Hokuyo_lidar lidar;

    cSerial PH2_sp(PH2_PORT, PH2_BAUD);
    PH2_sp.flush();
    int a = 0;

    while (1)
    {
        char b = (char) PH2_sp.Getchar();

        if (a)
        {
          PH2_sp.puts("$1234-0089+1#");
          a = 0;
         }
         else
         {
             PH2_sp.puts("$0234+1089-0#");
             a = 1;
         }
        cout << "Getting " << b << endl;

        delay(500);
    }

    cout << "Hello world! This is the end!" << endl;
    return 0;
}
