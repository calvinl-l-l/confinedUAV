#include "scheduler.h"
std::mutex mux_rx, mux_msp, mux_lidar;
std::condition_variable cond_rx;
std::condition_variable cond_lidar;
int ui = 0;



void RC_control_thread(void* p1, void* p2, void* p3)
{
    Pixhawk_Interface *quad = (Pixhawk_Interface*) p1;
    Hokuyo_lidar *lidar = (Hokuyo_lidar*) p2;
    position_controller* fc = (position_controller*) p3;

/*

transimtter switch diagram:
sw5 = mode
sw6 = auto
sw7 = arm

    5
    ---------------
    |  6       7  |
    |             |
    |             |
    |             |
    ---------------
*/
    while (1)
    {
        // RC CHANNELS OVERRIDE ---------------------------------------------

        unsigned long t0 = micros();

        //quad->update_arm_status();

        fc->update_controller_input(lidar->pos_loc_y, lidar->pos_loc_z, lidar->dist_wallL, lidar->dist_wallR, quad->ch6);

        int roll_out = fc->update_y_pos_controller();
        int thr_out = fc->update_z_pos_controller();

        // send mavlink here
        quad->write_RC_command(roll_out, 0,0);
        //cout << quad->ch1 << ' ' << quad->ch2 << ' ' << quad->ch3 << ' ' << quad->ch4 << ' ' << quad->ch5 << ' ' << quad->ch6 << ' ' << quad->ch7 << ' ' << endl;
        cout << fixed;
        cout << setprecision(3) << "lidar y" << lidar->pos_loc_y << " z " << lidar->pos_loc_z << " area " <<  lidar->area << " roll " << quad->ch2
             << " kp: " << fc->kp_pos_y << " ki: " << fc->ki_pos_y << " kd: " << fc->kd_pos_y << " kp_nw: " << fc->kp_pos_y_nw <<endl;
        // -----------------------------------------END RC CHANNELS OVERRIDE

        delay(10);

    } // end thread loop

}   // RC control thread


void read_sensors_thread(void *p1, void *p2, void *p3)
{
    Hokuyo_lidar *lidar = (Hokuyo_lidar*) p1;
    Pixhawk_Interface *quad = (Pixhawk_Interface*) p2;
    position_controller *fc = (position_controller*) p3;

    //quad->rx.ch7 = 1900;    // force init for file management

    while (1)
    {
        unsigned long t0 = millis();
        int  dt = 0;
        static unsigned long time_stamp = 0;
        int wr = 0;

        lidar->read();      // read lidar
        lidar->get_dist2wall(quad->roll);



        if (quad->ch6 > 1200)
        {
            time_stamp += 25;
            wr = 1;
        }
        else
        {
            wr = 0;
            time_stamp = 0;
        }

        write_data2file(wr, quad, lidar, fc, time_stamp);




        dt = millis() - t0;
        if (dt < 25)    delay(25-dt);

    }

    lidar->close();

}   // read lidar thread


void UI_thread(void* p)
{
    position_controller* fc = (position_controller*) p;

    while (1)
    {
        string input;
        cin >> input;

        if (input == "p")
        {
            cin >> input;
            fc->kp_pos_y = stof(input);
        }
        else if (input == "i")
        {
            cin >> input;
            fc->ki_pos_y = stof(input);
        }
        else if (input == "d")
        {
            cin >> input;
            fc->kd_pos_y = stof(input);
        }
        else if (input == "pnw")
        {
            cin >> input;
            fc->kp_pos_y_nw = stof(input);
        }


        delay(1000);
    }

}

void start_scheduler(Pixhawk_Interface &quad, Hokuyo_lidar &lidar, position_controller &fc)
{

    thread rc_control(RC_control_thread, &quad, &lidar, &fc);
    thread read_sensors(read_sensors_thread, &lidar, &quad, &fc);

    thread UI(UI_thread, &fc);
    //thread read_vehicle(read_vehicle_thread, &quad);
    //thread socket_com(socket_thread);               // socket communication
    //thread UI(UI_thread);                           // user interface


    rc_control.join();
    read_sensors.join();
    UI.join();

}



// ========== HELPER FUNCTIONS ======================

void write_data2file(int w, Pixhawk_Interface *Q, Hokuyo_lidar *L, position_controller *fc, unsigned long time_stamp)
{
    // w: 1 = start writing, 0 = close file
    static fstream  log_list;
    static ofstream ldata_log;
    static ofstream scan_log;   // needs to be outside

    int nlog;
    char filename[20];
    static int file_is_open = 0;


    if (w && !file_is_open)
    {
        log_list.open("log_list.txt");

        log_list >> nlog;
        log_list.close();
        log_list.open("log_list.txt", ios::out);
        log_list << ++nlog;
        log_list.close();

        snprintf(filename, sizeof filename, "ldata_%d.txt", nlog);
        ldata_log.open(filename);
        ldata_log << "PID values => kp: " << fc->kp_pos_y << " ki: " <<  fc->ki_pos_y << " kd: " << fc->kd_pos_y << endl;
        ldata_log << "Print out format:" << endl;
        ldata_log << "yc, zc, alt, CH1, CH3, is_manual?, roll, yaw, area, system time, RF, pitch, CH2, CH4" << endl;

        snprintf(filename, sizeof filename, "scan_%d.txt", nlog);
        scan_log.open(filename);
        file_is_open = 1;
        cout << "file created" << endl;
    }
    else if (w)
    {
        ldata_log << L->pos_loc_y << ',' << L->pos_loc_z << ',' << 0 << ',' << Q->ch4 << ',' << Q->ch1 << ',' << 1 << Q->roll << ',' << Q->yaw << ',' << L->area << ',' << time_stamp << ',' << 0 << ',' << Q->pitch << endl;

        for (int i=0;i<540*2;i++)
        {
            scan_log << i+1 << ',' << L->angle[i] << ',' << L->range[i] << ',' << time_stamp << endl;
        }

        //cout << "recording . . ." << endl;
    }
    else if (!w)
    {
        if (file_is_open)
        {
            ldata_log.close();
            scan_log.close();
            file_is_open = 0;
            cout << "files closed" << endl;
        }
    }


}


