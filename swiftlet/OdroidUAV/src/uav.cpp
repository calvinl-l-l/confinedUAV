#include <iostream>
#include "../lib/Scheduler/Scheduler.h"
#include "common.h"
#include "UI.h"
#include "lidar.h"
#include "messenger.h"

using namespace std;

#define PH2_MSG_LOOP_FREQ       150   // Hz
#define LOGGING_LOOP_FREQ       10
#define ARDUINO_COM_LOOP_FREQ   20

int main()
{
    // init variables
    const char* FS_PORT = "/dev/ttySAC0";         // fake sensor
    const char* arudino_PORT = "/dev/ttySAC2";    // Arduino
    int arduino_BAUD = 57600;
    int FS_BAUD = 921600;

    unsigned int max_n_threads = 4; // max number of concurrent tasks

    // init serial communicaitons
    cSerial FS_sp(FS_PORT, FS_BAUD);
    cSerial Ardu_sp(arudino_PORT, arduino_BAUD);
    FS_sp.flush();
    Ardu_sp.flush();

    // init class objects
    Bosma::Scheduler schedule(max_n_threads);
    Hokuyo_lidar lidar;
    messenger PH2(FS_sp);
    UI ui;

    // initialise system time
    lidar.set_startup_time(millis());
    ui.set_startup_time(millis());
    PH2.set_startup_time(millis());




//**************************************************************************
// scheduling tasks
//**************************************************************************
// PH2 messenger ---------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(1000/PH2_MSG_LOOP_FREQ), [&PH2]()
    {
        PH2.get_data();
    });


// lidar read ----------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(1), [&lidar, &FS_sp, &PH2, &ui]()
    {
        while (lidar.flag.init_startup_block)   {}  // wait for lidar to initialise

        unsigned long t0 = millis();    // for debug use

        lidar.read();

        lidar.get_ui_CMD(ui.lidar_CMD);
        ui.lidar_CMD.set_type = false;      // reset the CMD flag

        lidar.get_PH2_data(PH2.ph2_data);
        lidar.pos_update();

        PH2.send_pos_data(lidar.ldata);  // send position data to Pixhawk 2

        unsigned long dt_lidar = millis() - t0; // for debug use

    });

// Arduino com ---------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(1000/ARDUINO_COM_LOOP_FREQ), [&Ardu_sp, &PH2, &ui]()
    {
        Ardu_sp.putchar('m');
    });

// Logging + debug ------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(1000/LOGGING_LOOP_FREQ), [&ui, &lidar, &PH2]()
    {
        // logging
        if (ui.flag.log_data)
        {
            ui.start_log(lidar.ldata_q, PH2.ph2_data_q);
        }
        else
        {
            if (!ui.flag.file_is_closed)
            {
                ui.end_log();
                ui.flag.file_is_closed = true;
            }
        }

        // debug print
        if (ui.flag.debug_print)    ui.DEBUG_PRINT(lidar.ldata, PH2.ph2_data);
    });

// UI -------------------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(500), [&ui]()
    {
        // user interface input
        ui.run();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    });









// main while loop
    while (1)
    {
        delay(2000);
    }

    cout << "Hello world! This is the end gg!" << endl;
    std::this_thread::sleep_for(std::chrono::minutes(10));
}
