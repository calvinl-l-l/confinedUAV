#include "scheduler.h"


void RC_control_thread(void* p)
{
    MSP *quad = (MSP*) p;

    // Serial
    const char* arduino_port = "/dev/ttyACM1";
    const int arduino_baud = 115200;

    int data[16];
    int nbit = 0;
    int CH[8];
    // open port
    int ino_fd = serialOpen(arduino_port, arduino_baud);


    while (1)
    {
        // READ RX CHANNELS ---------------------------------------------
        // read RC input first
        while (serialDataAvail(ino_fd))
        {
            int inByte = serialGetchar(ino_fd);

            if (inByte == 91)   // ascii 91 = '[', 93 = ']'
            {
                nbit = 0;
            }
            else if ((inByte != 91) && (inByte != 93))
            {
                data[nbit++] = inByte;
            }
            else if ((nbit >= 16) && (inByte = 93))
            {
                break;  // break out if have one set of data;
            }
        }


        for (int i=0;i<8;i++)
        {
            CH[i] = (data[i*2] << 8 | data[i*2 + 1] ) & 0x07ff;

            // remapping channel values
            if (i==0)   CH[0] = val_remap(CH[0], 73, 1715, 1000, 2000);
            else        CH[i] = val_remap(CH[i], 343, 1706, 1100, 1900);
        }

        // --------------------------------------------END READ RX CHANNELS
        // RC CHANNELS OVERRIDE -------------------------------------------
        uint16_t msp_data[8];
        uint16_t dumb_data[8];

        msp_data[0] = CH[0];    // roll
        msp_data[1] = CH[1];    // pitch
        msp_data[2] = CH[2];    // yaw
        msp_data[3] = CH[3];    // throttle
        msp_data[4] = CH[4];
        msp_data[5] = CH[5];
        msp_data[6] = CH[6];
        msp_data[7] = CH[7];

        timeval starttime;
        timeval endtime;

        gettimeofday(&starttime,0);
        quad->send_msg(MSP_SET_RAW_RC, msp_data, 16, 'w');
        quad->send_msg(MSP_ATTITUDE, dumb_data, 0, 'r');
        quad->send_msg(MSP_RC, dumb_data, 0, 'r');

        quad->read_msg();
        gettimeofday(&endtime,0);


        // -----------------------------------------END RC CHANNELS OVERRIDE



        cout << quad->rcIn.ch1 << ' ' << quad->rcIn.ch2 << ' '<< quad->rcIn.ch3 << ' '<< quad->rcIn.ch4 << ' '<< quad->rcIn.ch5 << ' '<< quad->rcIn.ch6 << ' '<< quad->rcIn.ch7 << ' '<< quad->attitude.pitch << " time " << (endtime.tv_usec - starttime.tv_usec)/1 << endl;

        delay(10);
    } // end thread loop

}   // RC control thread


void read_lidar_thread(void *p)
{

    while (1)
    {


        delay(10);
    }

}   // read lidar thread


void socket_thread()
{

    while (1)
    {

    }

}   // socket communication

void UI_thread()
{

    while (1)
    {

    }

}   // user input thread

void start_scheduler(MSP &quad)
{

    std::thread rc_control(RC_control_thread, &quad);
    std::thread read_lidar(read_lidar_thread, &quad);
    //thread socket_com(socket_thread);               // socket communication
    //thread UI(UI_thread);                           // user interface

    rc_control.join();
    read_lidar.join();
}




