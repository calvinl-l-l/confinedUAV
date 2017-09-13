#include "scheduler.h"

int debug_print;

void RC_control_thread(void* p1, void* p2, void* p3)
{
    AP_interface *quad = (AP_interface*) p1;
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
        // RC CHANNELS OVERRIDE ***
        quad->write_RC_command(fc->roll_PWMout, 0,0);

        delay(10);

    } // end thread loop

}   // RC ontrol thread


void read_sensors_thread(void *p1, void *p2, void *p3)
{
    Hokuyo_lidar *lidar = (Hokuyo_lidar*) p1;
    AP_interface *quad = (AP_interface*) p2;
    position_controller *fc = (position_controller*) p3;

    unsigned long t0 = millis();

    mavlink_data_t qdata;

    while (1)
    {

        int  dt = 0;
        static unsigned long time_stamp = 0;
        static int wr = 0;
        static int p = 0;
        // sample pixhawk data ***
        qdata = quad->get_mavlink_data();

        // sample lidar data ***
        lidar->read(qdata.roll);      // read lidar

        //cout << qdata.roll << " ts: " << qdata.ts_attitude << endl;

        // Compute control output ***

        fc->update_controller_input(lidar->pos_loc_y2, lidar->pos_loc_z2, lidar->dist_wallL, lidar->dist_wallR, qdata.ch6, qdata.ch8);
        fc->flag_outside_scan_boundary = lidar->lidar_check_outof_boundary();
        //fc->update_controller_input(quad->ch3 - 1500, lidar->alt.dist, lidar->dist_wallL, lidar->dist_wallR, quad->ch6);  // debug use onlyt

        fc->roll_PWMout = fc->update_y_pos_controller();

        //  Logging file ***
        if (qdata.ch6 > 1200)
        {
          // logging data or auto flight mode
          digitalWrite(22, HIGH); // trigger reset

          if (!wr)
          {
            quad->set_startup_time(); // reset
            lidar->set_startup_time();  // reset
            wr = 1;
          }

          time_stamp += 25;

        }
        else
        {
          // no logging + manual
          digitalWrite(22, LOW);
          wr = 0;
          time_stamp = 0;
        }

        signal_LED(fc->_flag_auto_mode, fc->flag_outside_scan_boundary);

        write_data2file(wr, qdata, lidar, fc);

        // Debug printout

        if (debug_print && p%4==0)
        {
          DEBUG_PRINT(qdata, lidar, fc);
          p = 0;
        }
        p++;

        dt = millis() - t0;

        if (dt < 25)    delay(25-dt);
        t0 = millis();
    } // end of while loop

    lidar->close();

}   // read lidar thread


void UI_thread(void* p1, void* p2)
{
    position_controller* fc = (position_controller*) p1;
    AP_interface* quad = (AP_interface*) p2;

    while (1)
    {
        string input;
        cin >> input;

        if (input == "p")
        {
            cin >> input;
            fc->kp_pos_y = stof(input);
            cout << "Current PID: " << fc->kp_pos_y << ' ' << fc->ki_pos_y << ' ' << fc->kd_pos_y << endl;
        }
        else if (input == "i")
        {
            cin >> input;
            fc->ki_pos_y = stof(input);
            cout << "Current PID: " << fc->kp_pos_y << ' ' << fc->ki_pos_y << ' ' << fc->kd_pos_y << endl;
        }
        else if (input == "d")
        {
            cin >> input;
            fc->kd_pos_y = stof(input);
            cout << "Current PID: " << fc->kp_pos_y << ' ' << fc->ki_pos_y << ' ' << fc->kd_pos_y << endl;
        }
        else if (input == "pnw")
        {
            cin >> input;
            fc->kp_pos_y_nw = stof(input);
        }
        else if (input == "s")
        {
          debug_print = 0;
          //digitalWrite(22, HIGH); testing
        }
        else if (input == "stream")
        {
          cout << "start to stream data ..." << endl;
          debug_print = 1;
        }
        else if (input == "q")
        {
          //digitalWrite(22, LOW); testing
          quad->reboot();
        }
        else if (input == "setpt")
        {
          cin >> input;
          fc->pos_y_setpoint = stof(input);
          cout << "Lateral setpoint changed to: " << fc->pos_y_setpoint << endl;
        }
        else if (input == "trim")
        {
          cin >> input;
          fc->roll_trim = stoi(input);
          cout << "roll trim = " << fc->roll_trim << endl;
        }
        delay(1000);
    }

}   // end UI thread

void read_quad_thread(void *p)
{
    AP_interface* quad = (AP_interface*) p;
    quad->request_pixhawk_info_msg();

    while (1)
    {
        quad->read_msg();   // blocking function
    }

}   // end read FC thread

void start_scheduler(AP_interface &quad, Hokuyo_lidar &lidar, position_controller &fc)
{
    debug_print = 0;

    thread rc_control(RC_control_thread, &quad, &lidar, &fc);
    thread read_sensors(read_sensors_thread, &lidar, &quad, &fc);

    thread UI(UI_thread, &fc, &quad);
    thread read_quad(read_quad_thread, &quad);
    //thread socket_com(socket_thread);               // socket communication
    //thread UI(UI_thread);                           // user interface

    read_quad.join();
    rc_control.join();
    read_quad.join();
    UI.join();

}



// ========== HELPER FUNCTIONS ======================
void DEBUG_PRINT(mavlink_data_t qdata, Hokuyo_lidar *L, position_controller *fc)
{
    //for (int i=0; i<4;i++) cout << endl;


    cout << fixed << setprecision(3);
    cout << "pos_y " << setw(10) << L->pos_loc_y2 << " PWM " << setw(6) << fc->roll_PWMout
         << " CH_r " << qdata.ch1 << " p: " << fc->kp_pos_y << " i: " << fc->ki_pos_y
         << " d: " << fc->kd_pos_y << "    d-val "
         << setw(8) << fc->d_term << endl;

/*
    int map[2500];
    fill_n(map, 2500, 0);
    scan2pixelmap(L->y, L->z, L->pos_loc_y2, L->pos_loc_z2, map);

    for (int r=45;r>=10;r--){
      for (int c=0;c<50;c++)
      {
        if (map[r*50+c] > 1 && map[r*50+c] < 100) printf("o");
        else if (map[r*50+c] == 999)              printf("x");
        else if (map[r*50+c] == 9999)             printf("O");
        else                                      printf("  ");
      }
      cout << endl;
    }
*/

}



void write_data2file(int w, mavlink_data_t qdata, Hokuyo_lidar *L, position_controller *fc)
{
    // w: 1 = start writing, 0 = close file
    static fstream  log_list;
    static ofstream ldata_log;
    static ofstream scan_log;   // needs to be outside

    static ofstream yz_log;

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
        //ldata_log << "yc, zc, alt, CH2, CH1, is_manual?, roll, yaw, area, system time, RF, pitch, CH2, CH4" << endl;    //ch2 = roll ch3 = thr
	ldata_log << "pos_y, pos_z, CH_roll, control_PWMoutput, CH_thr, CH_8, roll, yaw, area, i-term, d-term, Ts_lidar, Ts_PH, pos_y3, battery" << endl;

        snprintf(filename, sizeof filename, "scan_%d.txt", nlog);
        scan_log.open(filename);


        snprintf(filename, sizeof filename, "yz_%d.txt", nlog);
        yz_log.open(filename);

        file_is_open = 1;
        cout << "file created" << endl;
    }
    else if (w)
    {
	/*
        ldata_log << L->pos_loc_y << ',' << L->pos_loc_z << ',' << L->alt.dist << ',' << qdata.ch2 << ',' << qdata.ch1 << ',' << 1 << ','
                  << qdata.roll << ',' << qdata.yaw << ',' << L->area << ',' << 0 << ',' << 0 << ',' << qdata.pitch
                  << ',' << L->pos_loc_y2 << ',' << L->pos_loc_z2 << ',' << L->ts << ',' << qdata.ts_attitude
                  << ',' << fc->i_term << ',' << fc->d_term << ',' << fc->ctrl_out << ',' << filtered_pos_y << endl;
*/
        ldata_log << fixed
        << setprecision(3) << setw(10)<< L->pos_loc_y2 << ',' << setw(10)<< L->pos_loc_z2 << ',' << qdata.ch1 << ',' << fc->roll_PWMout << ',' << qdata.ch3 << ',' << qdata.ch8 << ','
        << setw(10)<< qdata.roll << ',' << setw(10)<< qdata.yaw << ',' << setw(10)<< L->area << ',' << setw(10)<< fc->i_term << ',' << setw(10)<< fc->d_term << ',' << L->ts << ','
        << qdata.ts_attitude << ',' << L->pos_loc_y3 << ',' << qdata.batt_volt << endl;

        for (int i=0;i<540*2;i++)
        {
            scan_log << i+1 << ',' << L->angle[i] << ',' << L->range[i] << ',' << L->ts << endl;
        }

        for (int i =0; i < L->y.size();i++)
        {
          yz_log << i + 1 << ',' << L->y[i] << ',' << L->z[i] << ',' << L->nyz << endl;
        }


    }
    else if (!w)
    {
        if (file_is_open)
        {
            ldata_log.close();
            scan_log.close();
            yz_log.close();


            file_is_open = 0;
            cout << "files closed" << endl;
        }
    }


}
