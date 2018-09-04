#include <iostream>
#include "../lib/Scheduler/Scheduler.h"
#include "common.h"
#include "UI.h"
#include "lidar.h"
#include "messenger.h"
#include "localisation.h"

using namespace std;

#define PH2_MSG_LOOP_FREQ       180   // Hz
#define LOGGING_LOOP_FREQ       20
#define ARDUINO_COM_LOOP_FREQ   20


int main()
{
    // init variables
    const char* FS_PORT = "/dev/ttyUSB0"; //"/dev/ttySAC0";         // fake sensor
    const char* arudino_PORT = "/dev/ttySAC2";    // Arduino
    int arduino_BAUD = 57600;
    int FS_BAUD = 921600;

    unsigned int max_n_threads = 5; // max number of concurrent tasks

    // init serial communicaitons
    cSerial FS_sp(FS_PORT, FS_BAUD);
    cSerial Ardu_sp(arudino_PORT, arduino_BAUD);
    FS_sp.flush();
    Ardu_sp.flush();

    // init class objects
    Bosma::Scheduler schedule(max_n_threads);

    localisation HSM;

    messenger PH2(FS_sp);
    UI ui;

    // initialise system time
    HSM.set_startup_time(millis());
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
    schedule.interval(std::chrono::milliseconds(1), [&HSM, &FS_sp, &PH2, &ui]()
    {
        while (HSM.flag.init_startup_block)   {}  // wait for lidar to initialise

        unsigned long t0 = millis();    // for debug use

        HSM.read_scan();

        unsigned long t1 = millis();    // for debug use

        HSM.get_ui_CMD(ui.lidar_CMD);
        ui.lidar_CMD.set_type = false;      // reset the CMD flag

        HSM.get_PH2_data(PH2.ph2_data);

        HSM.run();

        HSM.data.nset++;
        PH2.send_pos_data(HSM.data);  // send position data to Pixhawk 2

        //cout << "li dt " << t1 - t0 << " HSM dt " << millis() - t0 << '\n';
        //cout << "pos y " << HSM.data.pos.y << " z " << HSM.data.pos.z << " dt " << millis() - t0 << '\n';
    });

// Arduino com ---------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(1000/ARDUINO_COM_LOOP_FREQ), [&Ardu_sp, &PH2, &ui]()
    {
        Ardu_sp.putchar('m');
    });

// Logging ------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(1000/LOGGING_LOOP_FREQ), [&ui, &HSM, &PH2]()
    {
        // get ch7 switch

        bool start_log = false;

        if (!ui.flag.log_data)
        {
            bool temp = PH2.get_log_switch();

            if (temp)   start_log = true;
            else        start_log = false;
        }
        else start_log = true;

        // logging
        if (start_log)
        {
            ui.start_log(HSM.data_q, PH2.ph2_data_q);
        }
        else
        {
            if (!ui.flag.file_is_closed)
            {
                ui.end_log();
                ui.flag.file_is_closed = true;
            }
        }

    });

// UI -------------------------------------------------------------------------
    schedule.interval(std::chrono::milliseconds(500), [&ui]()
    {
        // user interface input
        ui.run();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    });








// main while loop - handles debug print
    while (1)
    {
        // debug print
        if (ui.flag.debug_print)
        {
            cout << "y ";
            cout << HSM.data.pos.y;
            cout << " z ";
            cout << HSM.data.pos.z << ',';
            cout << " thrH ";
            cout << PH2.ph2_data.throttle_in << ',';
            cout << " u1 ";
            cout << PH2.ph2_data.u1 << ',';
            cout << " area ";
            cout << HSM.data.area;

            cout << '\n';
        }
        delay(1000/LOGGING_LOOP_FREQ);
    }

    cout << "Hello world! This is the end gg!" << endl;
    std::this_thread::sleep_for(std::chrono::minutes(10));
}
