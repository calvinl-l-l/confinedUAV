#ifndef UI_H
#define UI_H

#include "../rplidar_sdk/lidar_interface.h"
#include "../mavlink_lib/pixhawk_interface.h"
#include <fstream>


char PID_tuning(char uInput, float* kp, float* ki, float* kd);
void data_logging(std::ofstream& ldata_log, std::ofstream& scan_log, std::fstream& log_list, lidar_data ldata, pixhawk_data pixdata, long sys_time, float kp, float ki, float kd, int manual_ctrl, int RF, int rf2, int rf3);

#endif // UI_H
