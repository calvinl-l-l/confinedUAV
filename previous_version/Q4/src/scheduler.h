
#include <stdio.h>
#include "utility.h"
#include "MSP.h"
#include <iostream>
#include <wiringSerial.h>
#include <thread>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>


#define dt_RC       50
#define dt_lidar    25
#define dt_sensor   25
#define dt_sock     100
#define dt_UI       100

// common


// RC thread



using namespace std;

void start_scheduler(MSP &quad);

