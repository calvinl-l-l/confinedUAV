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
//#include <boost/lockfree/spsc_queue.hpp>
#include <queue>

#define LED_LOGIC_A 5
#define LED_LOGIC_B 27

using namespace std;


long val_remap(long x, long in_min, long in_max, long out_min, long out_max);
int median(vector<long> in);
void scan2pixelmap(vector<double> x, vector<double> y, double pos_x, double pos_y, int *map);

#endif
