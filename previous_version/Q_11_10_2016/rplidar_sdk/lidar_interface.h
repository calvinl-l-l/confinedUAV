/*
interface for using the rplidar
*/

#ifndef LIDAR_INTERFACE_H
#define LIDAR_INTERFACE_H

#include <string>
#include "include/rplidar.h"
#include "../custom_header/MedianFilter.h"
#include "../mavlink_lib/pixhawk_interface.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define dist_offset -40 // [mm] all range measurement have 40mm extra
#define buffer_size_header 60
#define buffer_size_data   30
#define buffer_size_action_header 30
#define buffer_size_PID_msg 25

using namespace rp::standalone::rplidar;
using namespace std;


typedef struct lidar_data{
    //raw data
    float dist[500];
    float angle[500];

    //raw -> cartesian
    float y[500];
    float z[500];
    float yz_angle[500];

    //number of point
    int nraw;
    int nyz;
    int prev_nyz;

    //centroid
    float yc;
    float zc;
    float prev_yc;
    float prev_zc;
    float area;
    float start_area;

    //temporary altitude
    float alt;
    float prev_alt;

}lidar_data;


void lidar_init(RPlidarDriver * drv, char* serial_port, _u32 baudrate);
void lidar_read(RPlidarDriver * drv, lidar_data* data, MedianFilter* yc, MedianFilter* zc);
bool checkRPLIDARHealth(RPlidarDriver * drv);
void getCentroid(lidar_data* data, MedianFilter* yc, MedianFilter* zc);
void getLidarInfo(RPlidarDriver * drv);
void lidar_alt(RPlidarDriver * drv, float roll, float pitch, lidar_data* data, int lidar_RNG);
int lidar_check_boundary(lidar_data data);

// socket com functions
int nChar(char* c);
void prepare_sock_msg_header(lidar_data ldata, pixhawk_data pixdata, double time_stamp, int mode, int RF, char* msg_header);
void prepare_sock_msg_data(lidar_data ldata, int idx, int data_end, char* msg_data);
void prepare_sock_action_header(string action, string param, char* msg_header);
void prepare_PID_msg(float kp, float ki, float kd, char* msg_header);



#endif
