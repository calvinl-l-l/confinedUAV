#ifndef _UI_H_
#define _UI_H_

#include "utility.h"
#include "../lib/cserial/cSerial.h"
#include "lidar.h"
#include "messenger.h"

struct UI_flag_t
{
    bool startup = false;
    bool file_is_opened = false;
    bool file_is_closed = true;
    bool log_data = false;
    bool reboot_PH2 = false;    // TODO

    // debug flags
    bool debug_print = false;
    bool debug_gain;
    int  debug_pos;     // 1:y  2:z  3:both
    int  debug_attitde;  // binary sum: 4 2 1, pitch roll yaw
};

class UI
{
public:
    ofstream info_log;
    ofstream lscan_log;

    unsigned int ts_PH2;
    unsigned int data_log_density;  // from (highest) 1-4 (lowest): 1080 542 360 270 pts
    int nlog;

    UI_CMD_t  lidar_CMD;
    UI_flag_t flag;

    UI();
    void set_startup_time(unsigned int sys_time);
    void init_log();
    void start_log(deque<lidar_data_t> ldata_q, deque<PH2_data_t> ph2_data_q);
    void end_log();
    void run();
    void DEBUG_PRINT();

private:
    string _input;

    unsigned int _ts_startup;
    UI_flag_t _flag;

    fstream _log_list;   // store log number

    void _set_log_num(int num);
    void _print_help();
};


#endif
