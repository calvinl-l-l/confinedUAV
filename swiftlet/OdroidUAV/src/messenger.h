#ifndef _MESSENGER_H_
#define _MESSENGER_H_

#include "common.h"
#include "../lib/cserial/cSerial.h"

#define DATA_MSG_BUF_SIZE 24    // number of variabale = 6
#define MAX_MESSENGER_DATA_QUEUE_SIZE 50

using namespace std;

class messenger
{
public:
    PH2_data_t ph2_data;
    deque<PH2_data_t> ph2_data_q;   // ring buffer

    messenger(cSerial sp);
    void send_pos_data(pos_data_t ldata);
    void get_data();
    void set_startup_time(unsigned int sys_time);

private:
    cSerial _sp;
    unsigned int _ts_startup;
    char _linebuf[DATA_MSG_BUF_SIZE];
    unsigned int _linebuf_len = 0;

    pos_data_t _ldata;

    mutex _msg_mtx;

    string _pos_msg_encoder();
};
#endif
