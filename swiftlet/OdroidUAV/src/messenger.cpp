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

    ph2_data.ts_PH2     = byte2int  (_linebuf, 0);
    ph2_data.roll       = byte2float(_linebuf, 1);
    ph2_data.pitch      = byte2float(_linebuf, 2);
    ph2_data.yaw        = byte2float(_linebuf, 3);
    ph2_data.ch.roll    = byte2int  (_linebuf, 4);
    ph2_data.ch.pitch   = byte2int  (_linebuf, 5);
    ph2_data.ch.thr     = byte2int  (_linebuf, 6);
    ph2_data.ch.yaw     = byte2int  (_linebuf, 7);
    ph2_data.ch.aux5    = byte2int  (_linebuf, 8);
    ph2_data.ch.aux6    = byte2int  (_linebuf, 9);
    ph2_data.ch.aux7    = byte2int  (_linebuf, 10);
    ph2_data.ch.aux8    = byte2int  (_linebuf, 11);
    ph2_data.u1         = byte2float(_linebuf, 12);
    ph2_data.throttle_in        = byte2float(_linebuf, 13);
    ph2_data.throttle_avg_max   = byte2float(_linebuf, 14);
    ph2_data.thr_hover          = byte2float(_linebuf, 15);
    ph2_data.perr.ez            = byte2float(_linebuf, 16);
    ph2_data.perr.dterm_z       = byte2float(_linebuf, 17);
    ph2_data.perr.iterm_z       = byte2float(_linebuf, 18);
    ph2_data.AC_alt_target      = byte2float(_linebuf, 19);
    ph2_data.AC_cr              = byte2float(_linebuf, 20);
    ph2_data.dist_err           = byte2float(_linebuf, 21);
    ph2_data.target_rangefinder_alt = byte2float(_linebuf, 22);
    ph2_data.perr.dtermfil_z       = byte2float(_linebuf, 23);

    // assign timestamp to data
    ph2_data.ts_odroid = millis() - _ts_startup;

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
//  $ is_healthy pos.y pos.z #
//  1       1      6     6    6  1 bytes, total = 18 bytes
    string msg = "";

    msg = "$";
    if (_ldata.is_healthy) msg += "1";
    else                   msg += "0";
    msg += int2str_ndigits(_ldata.pos.y, 5);
    msg += int2str_ndigits(_ldata.pos.z, 5);
    msg += int2str_ndigits(_ldata.nset, 6);
    msg += "#";

    //cout << "msg: " << msg << '\n';   // debug

    return msg;
}


bool messenger::get_log_switch()
{
    if ((ph2_data.ch.aux7 > 1500) && (ph2_data.ch.aux7 < 2100))    return true;
    else return false;
}

void messenger::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;
}
