#ifndef _UTILITY_
#define _UTILITY_

#include <wiringSerial.h>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <wiringPi.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include "../lib/cserial/cSerial.h"

#include <reckless/policy_log.hpp>      // data logging library
#include <reckless/file_writer.hpp>

#define LED_LOGIC_A 5
#define LED_LOGIC_B 27

using namespace std;

long val_remap(long x, long in_min, long in_max, long out_min, long out_max);
void signal_LED(int flag_auto_mode, int outside);
int median(vector<long> in);
void scan2pixelmap(vector<double> x, vector<double> y, double pos_x, double pos_y, int *map);


class UI
{
public:
    string input;

    ofstream info_log;
    ofstream lscan_log;

    UI();
    void init_log();
    void start_log();
    void end_log();

private:
    int _nlog;
    bool _flag_file_is_opened;

    fstream _log_list;   // store log number
};

#endif
