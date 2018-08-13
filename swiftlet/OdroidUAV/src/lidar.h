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

using namespace std;
using namespace qrk;

#define MAX_SCAN_AREA 10
#define LONELY_THRESHOLD 50;

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;


class Hokuyo_lidar
{
  public:
    long ts;
    long ts_startup;


    vector<long> range;
    vector<long> range_f;
    vector<double> angle;
    vector<double> angle_f;
    vector<double> y;
    vector<double> z;

    int nyz;
    double pos_loc_y;
    double pos_loc_z;
    double  pos_loc_y2;
    double pos_loc_z2;
    double pos_loc_y3;  // from dist2wall +y2

    float area;
    float start_area;

		struct altitude
		{
			float dist;
			char alt_ref;
		};
		altitude alt;


    float dist_wallR;  // distance to right wall
    float dist_wallL;  // distance to left wall


    Hokuyo_lidar();
    void set_startup_time();
    void read(float roll);
    void get_dist2wall(float roll);
    int lidar_check_outof_boundary();
    void get_altitude(char alt_ref, float roll);
    void sleep();
    void wake();
    void close();

  private:
    Urg_driver urg;
    int yz_start_pt;    // start point on the left, index 0
    int yz_end_pt;      // end point on the right, index end

    float _data_loss;

    int _flag_lidar_error;

    long t_temp;

    vector<int> lonely_pts_detector();
    void get_centroid1();
    void get_centroid2();
    void get_symmetry_pt();
};
#endif
