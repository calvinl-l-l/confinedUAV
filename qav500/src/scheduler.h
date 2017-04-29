#include <wiringPi.h>
#include <stdio.h>
#include <iostream>
#include <wiringSerial.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>
#include <iomanip>


#include "utility.h"
#include "AP_interface.h"
#include "lidar.h"
#include "AP_controller.h"

#define dt_RC       50
#define dt_lidar    25
#define dt_sensor   25
#define dt_sock     100
#define dt_UI       100

// common


// RC thread



using namespace std;

void start_scheduler(AP_interface &quad, Hokuyo_lidar &lidar, position_controller &fc);
void write_data2file(int w, mavlink_data_t qdata, Hokuyo_lidar *L, position_controller *fc);
void DEBUG_PRINT(mavlink_data_t qdata, Hokuyo_lidar *L, position_controller *fc);
