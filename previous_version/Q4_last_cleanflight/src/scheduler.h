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

#include "utility.h"
#include "MSP.h"
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

void start_scheduler(MSP &quad, Hokuyo_lidar &lidar, position_controller &fc);
void write_data2file(int w, MSP *Q, Hokuyo_lidar *L, unsigned long time_stamp);
