/*
 * pixhawk_interface.h
 *
 *  Created on: 20 Feb 2016
 *      Author: rbot
 */

#ifndef PIXHAWK_INTERFACE_H_
#define PIXHAWK_INTERFACE_H_

#include "mavlink/pixhawk/mavlink.h"
#include "mavlink/serial_port.h"
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <iostream>

#define COMMAND_ACK 77
#define SYSTEM_ID 255
#define COMP_ID 1

using namespace std;


//helper functions

//threads
void* start_PI_read_thread(void* args);
void* start_PI_heartbeat_thread(void* args);
//void* start_PI_read_rc_raw_thread(void* args);

//--------------------------------------------------
//            Pixhawk Interface Class
//--------------------------------------------------
class Pixhawk_Interface
{

public:
    //rc channels
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    int ch5;
    int ch6;
    int ch7;
    int ch8;

    int prev_ch5;

    //attitude in degree
    float pitch;
    float roll;
    float yaw;
	//Pichawk_Interface class
	Pixhawk_Interface();
	Pixhawk_Interface(Serial_Port *serial_port_);
	~Pixhawk_Interface();


	void read_msg();
	void send_heartbeat(mavlink_message_t msg);
	void write_RC_command(int roll, int thr, int yaw);

	//void request_RC_raw(mavlink_message_t);
    void request_pixhawk_info_msg();
    void get_home_pos();
    void set_target_alt(int alt);
    int update_arm_status();

	void start();
	void read_thread();
	void heartbeat_thread();
	void read_rc_raw_thread();

    //helper


private:

	Serial_Port *serial_port;

	pthread_t read_tid;
	pthread_t heartbeat_tid;
    //pthread_t read_rc_raw_tid;

    void arm_quad(mavlink_message_t msg, int arm);

};

#endif /* PIXHAWK_INTERFACE_H_ */
