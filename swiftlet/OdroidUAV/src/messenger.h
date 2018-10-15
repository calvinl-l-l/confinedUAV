#ifndef _MESSENGER_H_
#define _MESSENGER_H_

#include "common.h"
#include "../lib/cserial/cSerial.h"

#define N_MSG_VARIABLE                  19   // number of variabale in the messeage
#define DATA_MSG_BUF_SIZE               N_MSG_VARIABLE*4
#define MAX_MESSENGER_DATA_QUEUE_SIZE   50
#define TX2_MSG_BUF_SIZE 8  // for tx2

using namespace std;

class messenger
{
public:
    PH2_data_t ph2_data;
    deque<PH2_data_t> ph2_data_q;   // ring buffer

    pos_t pos_d;// from tx2
    pos_t prev_target;

    bool flag_start_OA;

    messenger(cSerial sp);
    void send_pos_data(pos_data_t ldata);
    void get_data();
    void get_tx2_data();
    void set_startup_time(unsigned int sys_time);
    bool get_log_switch();
    void get_OA_status(bool OA_status);
    void get_pos(pos_data_t ldata);
    
private:
    cSerial _sp;
    unsigned int _ts_startup;
    char _linebuf[DATA_MSG_BUF_SIZE];
    unsigned int _linebuf_len = 0;

    char _linebufTX2[TX2_MSG_BUF_SIZE];
    unsigned int _linebufTX2_len = 0;

    pos_data_t _ldata;

    char _pID;  // tx2
    mutex _msg_mtx;
    mutex _msg_tx2_mtx;

    vector<unsigned char> _pos_msg_encoder();

};
#endif
