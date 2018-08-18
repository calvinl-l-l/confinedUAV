#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <wiringSerial.h>
#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <stdio.h>
#include <deque>

#define LED_LOGIC_A 5
#define LED_LOGIC_B 27

using namespace std;

//=============================================================================
// lidar struct
//=============================================================================
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
    int dist2wallL;     // unused
    int dist2wallR;     // unused
    int dist2floor;     // unused
    int dist2ceiling;   // unused
    int alt;

    float area;

    bool is_healthy = true;
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

//=============================================================================
// messenger struct
//=============================================================================
struct PH2_data_t
{
    unsigned int ts_PH2;
    unsigned int ts_odroid;

    float roll;  // in rad
    float pitch;
    float yaw;

    // control output
};

long val_remap(long x, long in_min, long in_max, long out_min, long out_max);
int median(vector<long> in);
void scan2pixelmap(vector<double> x, vector<double> y, double pos_x, double pos_y, int *map);
string int2str_5digits(int value);
unsigned int ndigit(int value);

#endif
