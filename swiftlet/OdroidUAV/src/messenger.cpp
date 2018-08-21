#include "messenger.h"

messenger::messenger(cSerial sp)
{
    _sp = sp;
}

void messenger::get_data()
{
    char c;
    _linebuf_len = 0;

    if (_sp.DataAvail())
    {
        c = _sp.Getchar();

        if (c == '$')
        {
            for (int i=0; i<DATA_MSG_BUF_SIZE; i++)
            {
                _linebuf[_linebuf_len++] = _sp.Getchar();
            }
        }
    }

    lock_guard<mutex>   lock(_msg_mtx); // protecting data writing
    // decoding message
    int r           = byte2int(_linebuf, 0);
    int p           = byte2int(_linebuf, 1);
    int y           = byte2int(_linebuf, 2);
    ph2_data.ts_PH2 = byte2int(_linebuf, 3);

    ph2_data.roll   = (float) r/1000.0f;
    ph2_data.pitch  = (float) p/1000.0f;
    ph2_data.yaw    = (float) y/1000.0f;

    ph2_data.my_cr = byte2int(_linebuf, 4);
    ph2_data.ac_cr = byte2int(_linebuf, 5);
    int temp_alt_target        = byte2int(_linebuf, 6);
    ph2_data.alt_target = (float) temp_alt_target/1000.0f;
    int temp_alt        = byte2int(_linebuf, 7);
    // assign timestamp to data
    ph2_data.ts_odroid = millis() - _ts_startup;

    printf("roll %.2f, myCR %d, acCR %d, alt_tar %.4f, alt_cm: %d, ts: %d \n", ph2_data.roll*180/M_PI, ph2_data.my_cr, ph2_data.ac_cr, ph2_data.alt_target, temp_alt, ph2_data.ts_PH2);

    // pushing data to ring buffer
    if (ph2_data_q.size() >= MAX_MESSENGER_DATA_QUEUE_SIZE) ph2_data_q.pop_front();
    ph2_data_q.push_back(ph2_data);
}

void messenger::send_pos_data(pos_data_t ldata)
{
    _ldata = ldata;

    string msg;
    msg = _pos_msg_encoder();

    for (int i=0; i<msg.length(); i++)
    {
        _sp.putchar(msg[i]);
    }
}

string messenger::_pos_msg_encoder()
{
//=================================
//********  MSG FORMAT  ***********
//=================================
//
//  message:
//  $ is_healthy pos.y pos.z alt #
//  1       1      6     6    6  1 bytes, total = 18 bytes
    string msg = "";

    msg = "$" + to_string(_ldata.is_healthy);
    msg += int2str_5digits(_ldata.pos.y);
    msg += int2str_5digits(_ldata.pos.z);
    msg += int2str_5digits(_ldata.alt);
    msg += "#";

    return msg;
}



void messenger::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;
}
