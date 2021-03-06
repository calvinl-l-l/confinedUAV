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
#include <mutex>
#include <iomanip>

#define LED_LOGIC_A 5
#define LED_LOGIC_B 27

using namespace std;

//=============================================================================
// localisation + lidar struct
//=============================================================================
struct pos_t
{
    int y = 0;	// mm
    int z = 0;	// mm
};

// lidar data type
struct pos_data_t
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
    int nset = 0;         // nth data set

    // units in mm and degree
    vector<long> range;
    vector<double> angle;   // angle in radian
    vector<int> pc_y;
    vector<int> pc_z;

    int nyz = 0;

    pos_t pos;

    // distance from boundaries, unit: mm
    int dist2wallL;     // unused
    int dist2wallR;     // unused
    int dist2floor;     // unused
    int dist2ceiling;   // unused
    int alt = 0;

    float area = 0;

    bool is_healthy = true;
};

// altitude location: from roof or from ground
enum lidar_alt_type
{
    FLOOR = 0,
    ROOF,
    TUNNEL
};

// lidar flags
struct lidar_flag_t
{
    bool lidar_error;
    bool outof_boundary;
    bool printed_alt_mode;        // whether alt mode is printed
    bool init_startup_block;    // block lidar thread until false
    bool tunnel_init;
    
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
// control output
struct pos_error_t
{
    float ez = 0;
    float dterm_z = 0;
    float dtermfil_z = 0;
    float iterm_z = 0;
};

struct rc_channel_t
{
    int roll  = 1500;
    int pitch = 1500;
    int yaw   = 1500;
    int thr   = 0;
    int aux5  = 0;
    int aux6  = 0;
    int aux7  = 0;
    int aux8  = 0;
    int aux9  = 0;

};


struct PH2_data_t
{
    unsigned int ts_PH2;
    unsigned int ts_odroid;

    float roll = 0;  // in rad
    float pitch = 0;
    float yaw = 0;

    rc_channel_t ch;

    pos_t kf_pos;

    // control data
    float u1 = 0;
    float throttle_in = 0;
    float throttle_avg_max = 0;
    float thr_hover = 0;
    pos_error_t perr;
    float AC_alt_target;
    float AC_cr;
    float dist_err;
    float target_rangefinder_alt;
};

//============================================================================
// common
//============================================================================
union float_num
{
    unsigned char buf[4];
    float num;
};

union int_num
{
    unsigned char buf[4];
    int num;
};

long val_remap(long x, long in_min, long in_max, long out_min, long out_max);
int median(vector<long> in);
void scan2pixelmap(vector<double> x, vector<double> y, double pos_x, double pos_y, int *map);
string int2str_ndigits(int value, int nd);
unsigned int ndigit(int value);
int byte2int(char* buffer, int position);
float byte2float(char* buffer, int position);
int fsgn(float in);
int isgn(int in);

#endif
