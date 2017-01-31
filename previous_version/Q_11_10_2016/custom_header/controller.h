// Custom controller header
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <wiringPi.h>
#include "../rplidar_sdk/lidar_interface.h"
#include <iostream>
// PID gains
#define kp_alt 1
#define kp_hpos 1
#define kd_hpos 0.01

#define roll_offset 0

#define rc_llimit 1250 + roll_offset
#define rc_ulimit 1750 + roll_offset

#define integral_ulimit 150 + roll_offset
#define integral_llimit -150 + roll_offset

#define MAX_HEIGHT 1500 - 150
#define TARGET_ALT_ZONE 100
#define ALT_DELAY_OFFSET 20


void controller_init();
int alt_controller(lidar_data ldata, int manual_ctrl);
int horPos_controller(lidar_data ldata, float kp, float ki, float kd, int reset);
int auto_takeoff(lidar_data ldata, int* takeoff, int reset);
//int control_output_damper(int rc_out);




#endif // CONTROLLER_H
