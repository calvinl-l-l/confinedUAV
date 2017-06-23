#ifndef AP_INTERFACE_H_
#define AP_INTERFACE_H_

#include "../include/mavlink/c_library_v1/common/mavlink.h"
#include "mavlink_sp.h" 
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <iostream>
#include <boost/lockfree/spsc_queue.hpp>
#include "utility.h"

#define COMMAND_ACK 77
#define SYSTEM_ID 255
#define COMP_ID 1

using namespace std;

struct mavlink_data_t
{
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


    unsigned long ts_startup;
    unsigned long ts_attitude;
    unsigned long ts_CH;
};


class AP_interface
{
public:


    // functions
    AP_interface(Serial_Port *serial_port_);
    void read_msg();
    mavlink_data_t get_mavlink_data();
    void send_heartbeat();
    void write_RC_command(int roll, int thr, int yaw);
    void request_pixhawk_info_msg();
    void set_startup_time();
    void arm(int is_arm);
    void close_sp();

private:
	Serial_Port *serial_port;
    //bool reading_status;
    mavlink_data_t mavlink_data;
    //boost::lockfree::spsc_queue <mavlink_data_t, boost::lockfree::capacity<1024>> mavlink_data_q;
    boost::lockfree::spsc_queue <mavlink_data_t, boost::lockfree::capacity<512> > mavlink_data_q;

    unsigned long ts;
};


#endif
