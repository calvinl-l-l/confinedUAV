#ifndef _LIDAR_H_
#define _LIDAR_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "urg_cpp/Lidar.h"
#include "urg_cpp/Urg_driver.h"
#include <iomanip>
#include "common.h"
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


// lidar class
class Hokuyo_lidar
{
public:
    lidar_data_t ldata;
    deque<lidar_data_t> ldata_q;    // data ring buffer

    lidar_flag_t flag;

    Hokuyo_lidar(); //TODO: need to inherent localisation class
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
