#ifndef _LIDAR_
#define _LIDAR_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "urg_cpp/Lidar.h"
#include "urg_cpp/Urg_driver.h"
#include <iomanip>
#include "utility.h"
#include "messenger.h"

using namespace std;
using namespace qrk;

#define MAX_SCAN_AREA 10
#define LONELY_THRESHOLD 50;

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;

// lidar data type
struct lidar_data_t
{
    unsigned int ts_odroid;    // odroid system timestamp
    long         ts_lidar;     // lidar onboard ts

    // units in mm and degree
    vector<long> range;
    vector<double> angle;
    vector<double> pc_y;
    vector<double> pc_z;

    int nyz;

    float pos_y;
    float pos_z;

    // distance from boundaries, unit: mm
    int dist2wallL;
    int dist2wallR;
    int dist2floor;
    int dist2ceiling;

    float area;
};


// lidar class
class Hokuyo_lidar
{
public:
    lidar_data_t ldata;
    queue<lidar_data_t> ldata_q;

    bool flag_lidar_error;
    bool flag_outof_boundary;

    Hokuyo_lidar();
    void set_startup_time(unsigned int sys_time);   // in ms
    void read();
    void pos_update();
    void get_PH2_data(PH2_data_t data);
    void sleep();
    void wake();
    void close();

private:
    Urg_driver urg;
    int yz_start_pt;    // start point on the left, index 0
    int yz_end_pt;      // end point on the right, index end

    float _data_loss;

    unsigned int _ts_startup;
    float _start_area;

    PH2_data_t _ph2_data;

    //vector<int> lonely_pts_detector();
    //void _get_centroid1();
    void _get_centroid2();
    void _get_symmetry_pt();
    bool _lidar_check_outof_boundary();
};
#endif
