// Standard include s
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string>
#include <inttypes.h>
#include <fstream>
#include <iomanip>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <pthread.h>
#include <wiringPi.h>
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

//Custom includes
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "mavlink_lib/common/mavlink.h"
#include "mavlink_lib/serial_port.h"
#include "mavlink_lib/pixhawk_interface.h"
#include "rplidar_sdk/include/rplidar.h"
#include "rplidar_sdk/lidar_interface.h"
#include <pca9685.h>
#include "custom_header/controller.h"
#include "custom_header/MedianFilter.h"
#include "custom_header/UI.h"
#include "custom_header/sock_client.h"
#include "custom_header/RF_tag.h"

//LIDAR STUFF
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
//LIDAR STUFF

//struct timeval tv;		  ///< System time

using std::string;
using namespace std;
using namespace rp::standalone::rplidar;


//=================================================================
//---------------------- GLOBAL VARIABLES -------------------------
//-----------------------------------------------------------------

//--------------- Pixhawk device config ----------------
char *uart_name = (char*) "/dev/ttyACM0";
int baudrate = 115200;//57600;
int sys_id = 255;
int comp_id = 1;

int start_thread = 0;
int roll = 1000;
pixhawk_data pixdata;

int manual_ctrl = 1;
//--------------- Pixhawk device config ----------------

//----------------- RPLIDAR variable -------------------
char* lidar_serial_port = "/dev/ttyUSB0";
int   lidar_baudrate = 115200;

lidar_data range_data;
RPlidarDriver *lidar = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

int lsignal = 0; //0 = reading, 1 = finished 1 read loop
//----------------- RPLIDAR variable -------------------

//----------------- CONTROLLER variable -------------------
MedianFilter ldata_yc(10,0);
MedianFilter ldata_zc(10,0);

int hold_PID = 0;

//PID
float kp = 0.15;//0.15;
float ki = 0.01;//0.01;
float kd = 0.085;//0.065;
char tune = 'p';
//----------------- CONTROLLER variable -------------------

//----------- SOCKET COMMUNICATION variable ----------------
tcp_client sock_c;

string GS_input;
string GS_CMD;
ifstream file_list;
ifstream data_file;
//----------- SOCKET COMMUNICATION variable ----------------


//------------- RF TAG DISTANCE variable ---------------
int RF_distance1 = 0;
int RF_distance2 = 0;
int RF_distance3 = 0;

MedianFilter RF_dist_filter1(5,0);
MedianFilter RF_dist_filter2(5,0);
MedianFilter RF_dist_filter3(5,0);


//------------- RF TAG DISTANCE variable ---------------

//---------------------Other variables--------------------
//data logging

ofstream ldata_log;
ofstream scan_log;
fstream log_list;

//user input
char uInput;

//-----------------------------------------------------------------
//---------------------- GLOBAL VARIABLES -------------------------
//=================================================================


// minor helper functions
void mode_led(int man_ctrl);


//=================================================================
//--------------------------- THREADS -----------------------------
//-----------------------------------------------------------------

/*
pthread_t heartbeatThread;

void *heartbeat_thread(void *args)
{

	while (!start_thread){delay(10);}
	int sys_id = 25;
	int comp_id = 1;
	//Pixhawk_Interface p(serial_port1, sys_id, comp_id);

	Pixhawk_Interface* p = (Pixhawk_Interface*) args;
	while (1)
	{

        long t = millis();

		if (roll > 2000)	roll = 1000;
		roll++;
		//p->read_msg();

		mavlink_message_t msg;
		p->send_heartbeat(msg);

		delay(1000);
		printf("<3 %ld\n", millis()-t );

    pixdata = p->request_pixhawkData();
	//printf("ch1 %d\n", pixdata.ch1);
	long t = millis();

	mavlink_message_t msg;

    p->write_RC_command(msg, 0,0);//pixdata.ch1, pixdata.ch4);
	delay(10);
	//printf("time %ldms\n", millis() - t);
	}
}
*/
pthread_t sock_read_t;
void *sock_read_thread(void* arg)
{
    while (!sock_c.start_read_t) {}

    while (1)
    {
        GS_input = "";
        GS_input = sock_c.receive(30);
        while (GS_input.length() < 30)  GS_input += sock_c.receive(1);
        cout << "GS input " << GS_input << endl;
        if (GS_input.find("f_list_request")!=string::npos)      GS_CMD = "f_list_request";
        else if (GS_input.find("init_stream")!=string::npos)    GS_CMD = "init_stream";
        else if (GS_input.find("end_stream")!=string::npos)     GS_CMD = "end_stream";
        else if (GS_input.find("f_request")!=string::npos)
        {
            int t_idx = GS_input.find("t");
            int z_idx = GS_input.find("z");

            GS_CMD = "f_request" + GS_input.substr(t_idx +1, z_idx - t_idx - 2);
        }
        else if (GS_input.find("PID")!=string::npos)
        {
            int p_idx = GS_input.find("p");
            int i_idx = GS_input.find("i");
            int d_idx = GS_input.find("d");
            int end_idx = GS_input.find("-");

            piLock(2);
            kp = atof(GS_input.substr(p_idx + 1, i_idx - p_idx - 1).c_str());
            ki = atof(GS_input.substr(i_idx + 1, d_idx - i_idx - 1).c_str());
            kd = atof(GS_input.substr(d_idx + 1, end_idx - d_idx - 1).c_str());
            piUnlock(2);

            cout << "PID set to -> " << "kp = " << kp << " ki = " << ki << " kd = " << kd << endl;
        }

        cout << "GS msg: " << GS_CMD << endl;

        delay(1000);
    }
}


pthread_t sock_send_t;
void *sock_send_thread(void* arg)
{
    while (!start_thread){}
    //while (1) {}
    string server_ip_addr;
    string server_port;

    cout << "Please enter the IP addr: ";
    cin >> server_ip_addr;
    cout << "Please enter the port no.: ";
    cin >> server_port;

    //connect to host
    sock_c.conn(server_ip_addr , atoi(server_port.c_str()));

    delay(1000);  // wait a while for connection

    int n = 0;
    // signalling server
    sock_c.send_data("start");

    // wait for server to response before streaming data
    while (1)
    {
        string init_cmd = sock_c.receive(7);
        if (init_cmd.find("start")!=string::npos)  break;
        delay(25);
    }

    sock_c.start_read_t = 1;

    // init PID send
    char PID_msg_header[buffer_size_PID_msg];
    prepare_PID_msg(kp, ki, kd, PID_msg_header);
    sock_c.send_data(PID_msg_header);

    cout << "started data stream" << endl;

    double sys_time = 0;

    // start streaming data
    while (1)
    {

        //send some data
        char msg_header[buffer_size_header];
        char msg_data[buffer_size_data];
        char msg_action_header[buffer_size_action_header];

        string action;
        string param;
        int stream_data = 0;
        int data_end = 0;


        if (GS_CMD=="init_stream")
        {
            action = "stream";
            param  = "start";
            stream_data = 1;
        }
        else if (GS_CMD=="end_stream")
        {
            if (data_end)
            {
                stream_data = 0;
                action = "stream";
                param  = "stop";

                prepare_sock_action_header(action,param, msg_action_header);
                sock_c.send_data(msg_action_header);
            }

            if ((action=="stream")&&(param=="stop"))
            {
                action = "idle";
                param  = "NULL";

                prepare_sock_action_header(action,param, msg_action_header);
                sock_c.send_data(msg_action_header);

                GS_CMD = "";
            }
        }
        else if (GS_CMD=="f_list_request")
        {
            file_list.open("log_list.txt", ios::in);
            string line;

            if (file_list.is_open())
            {
                getline(file_list, line);
                file_list.close();
            }
            else
                cout << "Error openning file list" << endl;

            action = "file";
            param = "L" + line;
            prepare_sock_action_header(action,param, msg_action_header);
            sock_c.send_data(msg_action_header);

            cout << "file list msg: " << msg_action_header << endl;

            GS_CMD = "";
        }
        else if (GS_CMD.substr(0,5)=="f_req")
        {
            int file_no = atoi(GS_CMD.substr(GS_CMD.find("t")+1).c_str());
            cout << "file number: " << file_no << endl;

            action = "file";
            param = "send";
            prepare_sock_action_header(action,param, msg_action_header);
            sock_c.send_data(msg_action_header);

            // -------------- sending the ldata file
            string file_name = "scan_" + GS_CMD.substr(GS_CMD.find("t")+1) + ".txt";
            data_file.open(file_name.c_str(), ios::in);
            cout << "file name: " << file_name.c_str() << endl;

            string fline;

            if (data_file.is_open())
            {
                while (getline(data_file, fline))
                {
                    cout << "file line: " << fline << endl;
                    sock_c.send_data(fline + "\n");
                }

                data_file.close();

                sock_c.send_data("]");
            }
            else
                cout << "Error openning data file" << endl;

            // -------------- sending the scan file
            file_name = "ldata_" + GS_CMD.substr(GS_CMD.find("t")+1) + ".txt";
            data_file.open(file_name.c_str(), ios::in);

            if (data_file.is_open())
            {
                while (getline(data_file, fline))
                {
                    cout << "file line: " << fline << endl;
                    sock_c.send_data(fline + "\n");
                }

                data_file.close();

                sock_c.send_data("]");
            }
            else
                cout << "Error openning data file" << endl;


            // ------------- end of file transmission
            action = "file";
            param = "end";
            prepare_sock_action_header(action,param, msg_action_header);
            sock_c.send_data(msg_action_header);

            cout << "finished file transmission" << endl;

            GS_CMD = "";
        }

        // only start stream if requested
        if (stream_data)
        {
            prepare_sock_action_header(action, param, msg_action_header);
            sock_c.send_data(msg_action_header);

            //cout << "action header: " << msg_action_header << endl;

            // sending header
            piLock(0);
            prepare_sock_msg_header(range_data, pixdata, sys_time, manual_ctrl, RF_distance1, msg_header);

            cout << "header " << msg_header << endl;
            sock_c.send_data(msg_header);
            //cout << "msg sent" << endl;

            // sending lidar data
            for (int i=0;i<range_data.nraw;i++)
            {
                data_end = 0;
                if (i==range_data.nraw-1)   data_end = 1;
                prepare_sock_msg_data(range_data, i, data_end, msg_data);
                sock_c.send_data(msg_data);
                //delay(1);
            }
            piUnlock(0);
        }

        sys_time += 0.2;

        delay(200);
    }
}


pthread_t RF_t;
void *read_RF_tag_thread(void *args)
{
    while (!start_thread){}

    int rf_fd = RF_serial_init();

    char rf_buff[255];

    while (1)
    {
        int n = read(rf_fd, rf_buff, 255);
        rf_buff[n] = 0;

        string rf_out = string(rf_buff);

        piLock(0);
        if (rf_out.find("mr")!=string::npos)
        {
            int mr_i = rf_out.find("mr");

            int dist = strtol( rf_out.substr(9,5).c_str(), NULL, 16);
            int dist2 = strtol( rf_out.substr(20,3).c_str(), NULL, 16);
            int dist3 = strtol( rf_out.substr(29,3).c_str(), NULL, 16);
		
		cout << dist << "   string " << strtol(rf_out.substr(9,5).c_str(), NULL,16) << endl;
            if (dist!=0)    RF_distance1 = dist;
            if (dist2!=0)    RF_distance2 = dist2;
            if (dist3!=0)    RF_distance3 = dist3;
        }
        int FIL_RF_dist1 = RF_dist_filter1.in(RF_distance1);
        int FIL_RF_dist2 = RF_dist_filter2.in(RF_distance2);
        int FIL_RF_dist3 = RF_dist_filter3.in(RF_distance3);
	
	//cout << RF_distance1 << "   string " << rf_out.substr(11,3).c_str() << endl;
        RF_distance1 = FIL_RF_dist1;
        RF_distance2 = FIL_RF_dist2;
        RF_distance3 = FIL_RF_dist3;

        piUnlock(0);


        //cout << "distance = " << RF_distance1 << " filtered = " << FIL_RF_dist << endl;

        delay(100);
    }


}

int testc = 0;
pthread_t rc_control_t;

void *rc_control_thread(void *args)
{
    while (!start_thread){}

    Pixhawk_Interface* p = (Pixhawk_Interface*) args;
    int trans_roll = 1500 + roll_offset;
    int roll_left = 1; // left is +ve, right is -ve, 1 means roll to left
    int reset = 1;
    int roll_out = 1500;
    int thr_out = 0;

    static int rc_timer = 1;

    while (1)
    {
        // reseting controller - reset while in manual
        if (!manual_ctrl)   reset = 0;
        else reset = 1;
        //reset = 0;
        if (lsignal)    // only run when new data comes in
        {
            roll_out = horPos_controller(range_data,kp,ki,kd,reset);//range_data.yc);

            thr_out = alt_controller(range_data, manual_ctrl);

            lsignal = 0;    // reset the signal - ack lidar data
        }
        // transistion switch between mode
        if ((pixdata.ch5!=pixdata.prev_ch5)&&(pixdata.prev_ch5 > 1700)&&!hold_PID)
        {
            hold_PID = 1;

            if (trans_roll>roll_out)
                roll_left = 0;
            else
                roll_left = 1;

        }

        if (hold_PID&&!manual_ctrl)
        {
            if (roll_left)
            {
                trans_roll += 10;

                if (trans_roll >= roll_out) hold_PID = 0;
            }
            else
            {
                trans_roll -= 10;

                if (trans_roll <= roll_out) hold_PID = 0;

            }

            roll_out = trans_roll;
        }
        else
            trans_roll = 1500 + roll_offset;

        // writing RC override command
        mavlink_message_t msg;
        if (!manual_ctrl)
        {
            p->write_RC_command(msg, (int) roll_out,thr_out,0);
        }
        else
        {
            p->write_RC_command(msg,0,0,0);
        }

        if (rc_timer > 8)
        {
            rc_timer = 1;

            //cout << "alt " << range_data.alt << " yc " << range_data.yc<<" zc "<<range_data.zc<<" ch1 "<<pixdata.ch1<<" THR "<< thr_out<<" Area " << range_data.area << " startA " << range_data.start_area<<endl;

        }
        else
        {
            rc_timer++;
        }
        //cout << "nyz: " << range_data.nyz << " nraw: " << range_data.nraw << " yc: " << range_data.yc << " zc: " << range_data.zc << endl;
        //printf("%d %d %d %d %d %d %d %d\n", pixdata.ch1, pixdata.ch2, pixdata.ch3, pixdata.ch4, pixdata.ch5, pixdata.ch6, pixdata.ch7, pixdata.ch8);


        delay(20);
    }
}

pthread_t lidarThread;
//
void *read_lidarThread(void *args)
{
    while (!start_thread){}
    long sys_time = 0;
    while (1)
    {

        long t = millis();

        piLock(0);
        lidar_read(lidar, &range_data, &ldata_yc, &ldata_zc);

        // compute altitude
        lidar_alt(lidar, pixdata.roll, pixdata.pitch, &range_data, 10);
        piUnlock(0);
        // compute centroid
        //getCentroid(&range_data);


        //------------------------------ dynamic time delay
        long t_delay = millis() - t;

        if (t_delay>=100)
            {}
        else
        {
            delay(100 - t_delay);
            t_delay = 100;
        }
        t_delay = millis() - t;

        lsignal = 1;  // signal read lidar data

        //printf("time delay: %ld\n", t_delay);
        //-------------------------------- dynamic time delay



        //data logging
        sys_time += t_delay;


        // log data
        data_logging(ldata_log, scan_log, log_list, range_data, pixdata, sys_time, kp, ki, kd, manual_ctrl, RF_distance1, RF_distance2, RF_distance3);

    }
}// end of read lidar thread

pthread_t userThread;

void *userInputThread(void *args)
{

    while (1)
    {
        //scanf("%c", &uInput);
        //printf("%c\n", uInput);

        //PID_tuning(uInput, &kp, &ki, &kd);
        delay(250);
    }
}
//-----------------------------------------------------------------
//--------------------------- THREADS -----------------------------
//=================================================================

//--------------------------  MAIN --------------------------------
int main(int argc, char **argv)
{

	printf("Starting oboard system\n");

    wiringPiSetup();
    pinMode(0,OUTPUT);  // LED init

    //PWM driver setup
    int pwm_fd = pca9685Setup(PWM_BASE, PWM_driver_addr, PWM_FREQ);
    pca9685PWMReset(pwm_fd);

    lidar2pixhawkALT(12, 60);

    printf("Power on flight controller now\nPlease enter 'y' to continue: ");
    while (1)
    {
        char c;
        scanf("%c", &c);
        if (c=='y') break;
    }
    printf("Connecting to flight controller and lidar...\n");
    //Pixhawk setup----
	parse_commandline(argc, argv, uart_name, baudrate);
	//Serial port setup
	Serial_Port serial_port(uart_name, baudrate);
	Pixhawk_Interface pilot(&serial_port, sys_id, comp_id);
	serial_port.start();
	pilot.start();
    //end Pixhawk setup

    //lidar setup
    lidar_init(lidar, lidar_serial_port, lidar_baudrate);

	//threading stuff
    if (pthread_create(&lidarThread, NULL, read_lidarThread, NULL)) printf("Error creating lidar thread\n");
    if (pthread_create(&userThread, NULL, userInputThread, NULL)) printf("Error creating lidar thread\n");
    if (pthread_create(&rc_control_t, NULL, rc_control_thread, &pilot)) printf("Error creating RC control thread\n");
    if (pthread_create(&sock_send_t, NULL, sock_send_thread, NULL)) printf("Error creating socket send thread\n");
    if (pthread_create(&sock_read_t, NULL, sock_read_thread, NULL)) printf("Error creating socket read thread\n");
    if (pthread_create(&RF_t, NULL, read_RF_tag_thread, NULL)) cout << "Error creating RF tag read thread" << endl;


	mavlink_message_t msg_rc;
	//pilot.arm_quad(msg_rc, 1);

    pilot.request_pixhawk_info_msg();
    //pilot.get_home_pos();
    //pilot.set_target_alt(1);
    printf("starting ... 1\n");
    delay(1000);
    printf("starting ... 2\n");
    delay(1000);
    printf("starting ... 3\n");

    start_thread = 1;
    printf("Onboard system started!\n");
    delay(100);
    range_data.start_area = range_data.area;






	while (1)//----------- LOOP -------------------------
	{


		//int d = 60;
        //scanf("%d", &d);

        lidar2pixhawkALT(12, range_data.alt/10);
        pixdata = pilot.request_pixhawkData();



        if (pixdata.ch6 > 1700)
        {
            mavlink_message_t arm_msg;
            pilot.arm_quad(arm_msg, 0); //disarm the X8

            // manual
            manual_ctrl = 1;

        }
        else if ((pixdata.ch6 < 1700)&&(pixdata.ch6 > 1300))
        {
            // auto mode (else override)
            if (pixdata.ch5 > 1700)
            {
                manual_ctrl = 1;

            }
            else
            {
                manual_ctrl = 0;

            }
        }
        else if (pixdata.ch6 < 1300)
        {
            // manual mode
            mavlink_message_t msg;
            pilot.write_RC_command(msg,0,0,0);

            manual_ctrl = 1;

        }

        // checking if out of boundaries
        if (!manual_ctrl)
        {

            if (!lidar_check_boundary(range_data))
            {
                manual_ctrl = 1;
            }

        }

        mode_led(manual_ctrl);

	}// -------------- END OF LOOP ------------------------


	//THE END - DO NOT COME HERE
	serial_port.stop();
	return 0;
}//----------------------- END OF MAIN --------------------------------













// minor helper functions
void mode_led(int man_ctrl)
{
    if (!man_ctrl)
        digitalWrite(0,1);  // auto mode led on
    else
        digitalWrite(0,0);  // manual mode led off
}
