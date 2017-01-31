#include "scheduler.h"
std::mutex mux_rx, mux_msp, mux_lidar;
std::condition_variable cond_rx;
std::condition_variable cond_lidar;
int ui = 0;



void RC_control_thread(void* p1, void* p2, void* p3)
{
    MSP *quad = (MSP*) p1;
    Hokuyo_lidar *lidar = (Hokuyo_lidar*) p2;
    position_controller* fc = (position_controller*) p3;



    while (1)
    {
        // RC CHANNELS OVERRIDE ---------------------------------------------

        unsigned long t = micros();

        std::unique_lock<std::mutex> lock_lidar(mux_lidar);
        cond_lidar.wait(lock_lidar);
            fc->update_pos(lidar->pos_y, lidar->alt_floor, 0, lidar->alt_floor);
        lock_lidar.unlock();

        fc->get_target_alt(quad->rx.ch4, quad->rx.ch7);

        // compute alt hold control output
        //fc.update_z_controller(quad->msp_rc_data, quad->rx.ch4);


        std::unique_lock<std::mutex> lock_rx(mux_rx);
        cond_rx.wait(lock_rx);

        fc->simple_PID_altHold(quad->msp_rc_data);
        //cout << "  alt: " << lidar->alt_floor << ' ' << quad->rcIn.ch4 << endl;
        cout << "p" << fc->kp_pos_z << 'i' << fc->ki_pos_z << 'd' << fc->kd_pos_z << endl;

            std::unique_lock<std::mutex> lock_msp(mux_msp);
                quad->send_msg(MSP_SET_RAW_RC, quad->msp_rc_data, 16, 'w');
            lock_msp.unlock();
        lock_rx.unlock();


      //  cout << "time " << micros()-t <<endl;
        // -----------------------------------------END RC CHANNELS OVERRIDE

        delay(10);

    } // end thread loop

}   // RC control thread


void sample_rx_thread(void* p1, void* p2)
{
    MSP* quad = (MSP*) p1;
    Hokuyo_lidar* lidar = (Hokuyo_lidar*) p2;

    const char* ardu_port = "/dev/ttyACM0";
    const int ardu_baud = 115200;
    int ardu_data[18];
    int CH[8] = {1500,1500,1500,1000,1900,1900,1900,1900};
    static int nbit = 0;
    static int new_frame = 0;
    MedianFilter alt_filter(10,0);

    int ardu_fd = serialOpen(ardu_port, ardu_baud);
    if (!ardu_fd)   cout << "error connecting to arduino" << endl;


    while (1)
    {
        while (serialDataAvail(ardu_fd))
        {
            int ibyte = serialGetchar(ardu_fd);

            if (ibyte==91)  // ascii 91 = '[', 93 = ']'
            {
                nbit = 0;
                new_frame = 1;
            }
            else if (ibyte==93)
            {
                new_frame = 0;

                // decoding data into raw channel values
                for (int i=0;i<8;i++)
                {
                    CH[i] = (ardu_data[i*2] << 8 | ardu_data[i*2 + 1] ) & 0x07ff;

                    // remapping channel values
                    if (i==0)
                    {
                        CH[0] = val_remap(CH[0], 73, 1715, 1000, 2000);
                    }
                    else
                    {
                        CH[i] = val_remap(CH[i], 343, 1706, 1100, 1900);
                    }

                    //cout << "CH" << i << ": " << CH[i] << ' ';
                }


                lidar->alt_floor = to_discrete(alt_filter.in((ardu_data[16] << 8) + ardu_data[17]), 5);



                // call function to pass new channel values
                std::unique_lock<std::mutex> lock_rx(mux_rx);
                    quad->get_rx_CH(CH);
                lock_rx.unlock();
                cond_rx.notify_one();

            }
            else if (new_frame)
            {
                ardu_data[nbit++] = ibyte;
            }
        }

        delay(10);

    }
} // end sample rx thread

void read_sensors_thread(void *p1, void *p2)
{
    Hokuyo_lidar *lidar = (Hokuyo_lidar*) p1;
    MSP *quad = (MSP*) p2;

    quad->rx.ch7 = 1900;    // force init for file management

    while (1)
    {
        unsigned long t0 = millis();
        int  dt = 0;
        static unsigned long time_stamp = 0;

        // testing
        //std::unique_lock<std::mutex> lock_rx(mux_rx);
        //cout << quad->rx.ch1 << ' ' << quad->rx.ch2 << ' ' << quad->rx.ch3 << ' ' << quad->rx.ch4 << ' ' << quad->rx.ch5 << endl;

        //lock_rx.unlock();

        lidar->read();      // read lidar

        std::unique_lock<std::mutex> lock_msp(mux_msp);
        uint16_t dumb_data[8];
        quad->send_msg(MSP_RC, dumb_data, 0, 'r');
        lock_msp.unlock();

        std::unique_lock<std::mutex> lock_lidar(mux_lidar);
            quad->read_msg();
        lock_lidar.unlock();
        cond_lidar.notify_one();
        //cout << quad->rcIn.ch5 << ' ' << quad->rcIn.ch6 << ' ' << quad->rcIn.ch7 << ' ' << quad->rcIn.ch8 << ' ' << quad->rcIn.ch4 << endl;

        dt = millis() - t0;

        // writing to file
        int w = 0;
        if (quad->rx.ch7 < 1800)
        {
            w = 1;
            time_stamp += dt;
        }
        else
        {
            w = 0;
            time_stamp = 0;
        }

        //write_data2file(1, quad, lidar, time_stamp);




        //cout << "lidar " << lidar->range[540] << " time: " << dt << "  cin " << ui << endl;
        //delay(10);
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
            fc->kp_pos_z = stof(input);
        }
        else if (input == "i")
        {
            cin >> input;
            fc->ki_pos_z = stof(input);
        }
        else if (input == "d")
        {
            cin >> input;
            fc->kd_pos_z = stof(input);
        }


        delay(1000);
    }

}

void start_scheduler(MSP &quad, Hokuyo_lidar &lidar, position_controller &fc)
{

    thread rc_control(RC_control_thread, &quad, &lidar, &fc);
    thread read_sensors(read_sensors_thread, &lidar, &quad);
    thread sample_rx(sample_rx_thread, &quad, &lidar);
    thread UI(UI_thread, &fc);
    //thread read_vehicle(read_vehicle_thread, &quad);
    //thread socket_com(socket_thread);               // socket communication
    //thread UI(UI_thread);                           // user interface

    sample_rx.join();
    rc_control.join();
    read_sensors.join();
    UI.join();

}



// ========== HELPER FUNCTIONS ======================

void write_data2file(int w, MSP *Q, Hokuyo_lidar *L, unsigned long time_stamp)
{
    // w: 1 = start writing, 0 = close file
    static fstream  log_list;
    static ofstream ldata_log;
    static ofstream scan_log;   // needs to be outside

    int nlog;
    char filename[20];
    static int file_is_open = 0;

/*
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
        ldata_log << "PID values => kp: " << 0 << " ki: " <<  0 << " kd: " << 0 << endl;
        ldata_log << "Print out format:" << endl;
        ldata_log << "yc, zc, alt, CH1, CH3, is_manual?, roll, yaw, area, system time, RF, pitch, CH2, CH4" << endl;

        snprintf(filename, sizeof filename, "scan_%d.txt", nlog);
        scan_log.open(filename);
        file_is_open = 1;
        cout << "file created" << endl;
    }
    else if (w)
    {
        ldata_log << 0 << ',' << 0 << ',' << 0 << ',' << 0 << ',' << 0 << ',' << 1 << Q->attitude.roll << ',' << Q->attitude.yaw << ',' << 0 << ',' << time_stamp << ',' << 0 << ',' << Q->attitude.pitch << endl;

        for (int i=0;i<540*2;i++)
        {
            scan_log << i+1 << ',' << L->angle[i] << ',' << L->range[i] << ',' << time_stamp << endl;
        }

        cout << "recording . . ." << endl;
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
*/


    if (w && !file_is_open)
    {
        log_list.open("log_list.txt");

        log_list >> nlog;
        log_list.close();
        log_list.open("log_list.txt", ios::out);
        log_list << ++nlog;
        log_list.close();


        snprintf(filename, sizeof filename, "ch_log_%d.txt", nlog);
        scan_log.open(filename);
        file_is_open = 1;
        cout << "file created" << endl;
    }
    else if (w)
    {
        scan_log << Q->rx.ch1 << ',' << Q->rx.ch2 << ',' << Q->rx.ch3 << ',' << Q->rx.ch4 << ',' << Q->rx.ch5 << ',' << Q->rx.ch6 << ',' << Q->rx.ch7 << ',' << Q->rx.ch8 << ',' << Q->rcIn.ch1 << ',' << Q->rcIn.ch2 << ',' << Q->rcIn.ch3 << ',' << Q->rcIn.ch4 << ',' << Q->rcIn.ch5 << ',' << Q->rcIn.ch6 << ',' << Q->rcIn.ch7 << ',' << Q->rcIn.ch8 << endl;

        cout << "recording . . ." << endl;
    }
    else if (!w)
    {
        if (file_is_open)
        {

            scan_log.close();
            file_is_open = 0;
            cout << "files closed" << endl;
        }
    }

}

