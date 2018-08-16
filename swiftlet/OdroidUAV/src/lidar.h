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
#define LONELY_THRESHOLD 50

#define MAX_LDATA_QUEUE_SIZE 10

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;

// lidar data type
struct lidar_data_t
{
    unsigned int ts_odroid;    // odroid system timestamp
    long         ts_lidar;     // lidar onboard ts

    // units in mm and degree
    vector<long> range;
    vector<double> angle;   // angle in radian
    vector<double> pc_y;
    vector<double> pc_z;

    int nyz;

    int pos_y;  // in mm
    int pos_z;  // in mm

    // distance from boundaries, unit: mm
    int dist2wallL;
    int dist2wallR;
    int dist2floor;
    int dist2ceiling;
    int alt;


    float area;
};

// lidar flags
struct lidar_flag_t
{
    bool flag.lidar_error;
    bool flag.outof_boundary;
    lidar_alt_type alt;
};

// altitude location: from roof or from ground
enum lidar_alt_type
{
    FLOOR = 0,
    ROOF
}

// lidar class
class Hokuyo_lidar
{
public:
    lidar_data_t ldata;
    deque<lidar_data_t> ldata_q;    // data ring buffer

    lidar_flag_t flag;

    Hokuyo_lidar();
    void set_startup_time(unsigned int sys_time);   // in ms
    void read();
    void pos_update();
    void calc_alt(lidar_alt_type dir);
    void get_PH2_data(PH2_data_t data);
    void init_localisation();

    double deg2r(double degree);    // convert degree to radian;
    void sleep();
    void wake();
    void close();

private:
    Urg_driver urg;
    float _data_loss;
    float _start_area;
    unsigned int _ts_startup;

    PH2_data_t _ph2_data;       // data from Pixhawk 2
    // spectrum localisation
    vector<int> _specY_ref; // y reference spectrum
    vector<int> _specZ_ref; // z reference sepctrum
    vector<int> _specY_src; // y source spectrum
    vector<int> _specZ_src; // z source spectrum

    vector<int> _pt2spectrum(vector<double> point); // converting point to spectrum
    void _scan_matching_sptm(); //scan matching using spectrum analysis

    // centroid localisation
    void _get_centroid2();
    //void _get_symmetry_pt();
    //vector<int> lonely_pts_detector();
    //void _get_centroid1();
    //bool _lidar_check_flag.outof_boundary();
};
#endif
