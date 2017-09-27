#include <wiringSerial.h>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <wiringPi.h>
#include <vector>
#include <math.h>
#include <algorithm>

#define LED_LOGIC_A 5
#define LED_LOGIC_B 27

using namespace std;

long val_remap(long x, long in_min, long in_max, long out_min, long out_max);
void signal_LED(int flag_auto_mode, int outside);
int median(vector<long> in);
void scan2pixelmap(vector<double> x, vector<double> y, double pos_x, double pos_y, int *map);
