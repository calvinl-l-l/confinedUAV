#ifndef _UI_H_
#define _UI_H_

#include "utility.h"
#include "../lib/cserial/cSerial.h"
#include "lidar.h"
#include "messenger.h"


class UI
{
public:
    string input;

    ofstream info_log;
    ofstream lscan_log;

    unsigned int ts_PH2;

    UI();
    void set_startup_time(unsigned int sys_time);
    void init_log();
    void start_log(queue<lidar_data_t> ldata_q, queue<PH2_data_t> ph2_data_q);
    void end_log();
    void set_log_num(int num);

    void DEBUG_PRINT();

private:
    int _nlog;
    unsigned int _ts_startup;
    bool _flag_file_is_opened;

    fstream _log_list;   // store log number
};


#endif
