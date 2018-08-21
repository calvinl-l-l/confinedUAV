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
#define SCAN_DENSITY            3       // Npoint = 1080/SCAN_DENSITY, should only choose from 1 - 4

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;


// lidar class
class Hokuyo_lidar
{
public:
    pos_data_t data;
    deque<pos_data_t> data_q;    // data ring buffer

    lidar_flag_t flag;

    Hokuyo_lidar();
    void lidar_init();
    void set_startup_time(unsigned int sys_time);   // in ms
    void read_scan();
    void calc_alt();
    void get_PH2_data(PH2_data_t data);

    void set_max_scan_range(unsigned int range);
    void get_ui_CMD(UI_CMD_t in);
    void set_alt_type(lidar_alt_type dir);
    void print_alt_type();
    double deg2r(double degree);    // convert degree to radian;
    double r2deg(double radian);   // convert radian to degree;
    void sleep();
    void wake();
    void close();

protected:
    // general
    Urg_driver _urg;
    mutex _pos_mtx;
    unsigned int _max_scan_range;   // mm
    float _data_loss;
    float _start_area;
    unsigned int _ts_startup;

    unsigned int _offset_x = 250;   // mm
    unsigned int _offset_z = 30;

    PH2_data_t _ph2_data;       // data from Pixhawk 2
    UI_CMD_t   _cmd;            // command/data from UI

    // localisation
    int _tunnel_height;
    pos_data_t _data_ref;

    void _init_alt_type();
    void _update_pc();   // update point cloud
    void _save_ref_scan();
    void _get_centroid2();
    //void _get_symmetry_pt();
    //vector<int> lonely_pts_detector();
    //void _get_centroid1();
    //bool _lidar_check_flag.outof_boundary();
};
#endif
