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

#define MAX_SCAN_AREA           10      // not used for now
#define LONELY_THRESHOLD        50      // retired
#define ROOF_THRESHOLD          8000    // at least within 8m to be consider has roof
#define ALT_ROOF_ANGLE_RANGE    40      // in degree, total range
#define ALT_FLOOR_ANGLE_RANGE   20      // in degree
#define MAX_LDATA_QUEUE_SIZE    10
#define FRAME_HEIGHT            120     // from ground to lidar height

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;

// lidar data type
struct lidar_data_t
{
//  scan orientation
//
//   *** top view ***
//           *
//           *
//           *
//        *     *
//      *         *
//    +ve         -ve
//idx:1080          0
//
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

// altitude location: from roof or from ground
enum lidar_alt_type
{
    FLOOR = 0,
    ROOF,
    BOTH
};

// lidar flags
struct lidar_flag_t
{
    bool lidar_error;
    bool outof_boundary;
    bool printed_alt_mode;        // whether alt mode is printed
    bool init_startup_block;    // block lidar thread until false

    enum lidar_alt_type alt;
};

struct UI_CMD_t
{
    enum lidar_alt_type alt_type;
    bool set_type = false;

    // TODO: use string as a command instead, then have a decoder
    // TODO: command to set range
    // future commands
};

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
    void calc_alt();
    void get_PH2_data(PH2_data_t data);
    void init_localisation();

    void set_max_scan_range(unsigned int range);
    void get_ui_CMD(UI_CMD_t in);
    void set_alt_type(lidar_alt_type dir);
    void print_alt_type();
    double deg2r(double degree);    // convert degree to radian;
    double r2deg(double radian);   // convert radian to degree;
    void sleep();
    void wake();
    void close();

private:
    // general
    Urg_driver _urg;
    unsigned int _max_scan_range;   // mm
    float _data_loss;
    float _start_area;
    unsigned int _ts_startup;

    unsigned int _offset_x = 250;   // mm
    unsigned int _offset_z = 30;

    PH2_data_t _ph2_data;       // data from Pixhawk 2
    UI_CMD_t   _cmd;            // command/data from UI

    // altitude calc
    int _tunnel_height;

    void _init_alt_type();

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
