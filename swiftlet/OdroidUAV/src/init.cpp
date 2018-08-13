#include <iostream>
#include "../lib/Scheduler/Scheduler.h"
#include <wiringPi.h>
#include "lidar.h"
#include "utility.h"

using namespace std;


int main()
{
    // init variables
    char* FS_PORT = "/dev/ttySAC0";         // fake sensor
    char* arudino_PORT = "/dev/ttySAC2";    // Arduino
    int arduino_BAUD = 57600;
    int FS_BAUD = 921600;

    unsigned int max_n_threads = 4; // max number of concurrent tasks

    // init class objects
    Bosma::Scheduler schedule(max_n_threads);
    Hokuyo_lidar lidar;


    // init serial communicaitons
    cSerial FS_sp(FS_PORT, FS_BAUD);
    cSerial Ardu_sp(arudino_PORT, arduino_BAUD);
    FS_sp.flush();
    Ardu_sp.flush();


//**************************************************************************
// scheduling tasks
//**************************************************************************
// timer -----------------------------------------------------------------------
    schedule.every(std::chrono::milliseconds(50), [&FS_sp]()
    {
        //cout << "fs " << FS_sp.Getchar() << "  PH2 " << PH2_sp.Getchar() << endl;
        while (FS_sp.DataAvail())   cout << (char) FS_sp.Getchar();
        cout << endl;
    });


// lidar read ----------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(1), [&lidar, &FS_sp]()
    {
        unsigned long t0 = millis();

        lidar.read(0);
        //FS_sp.puts("$0232-1089-0#");

        unsigned long dt_lidar = millis() - t0;

    });

// Arduino com ---------------------------------------------------------------
    schedule.every(std::chrono::milliseconds(50), [&Ardu_sp]()
    {
        Ardu_sp.putchar('m');
    });

// UI -------------------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(50), []()
    {
        // user interface input
        string input;
        cin >> input;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    });

// Logging --------------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(100), []()
    {

    });

// main while loop
    while (1)
    {
        delay(2000);
    }

    cout << "Hello world! This is the end gg!" << endl;
    std::this_thread::sleep_for(std::chrono::minutes(10));
}

//PH2_sp.puts("$0234+1089-0#");
