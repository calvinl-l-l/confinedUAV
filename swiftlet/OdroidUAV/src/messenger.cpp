#include "messenger.h"

messenger::messenger(cSerial sp)
{
    _sp = sp;

    _pID = 0;

    pos_d.y = 0;
    pos_d.z = 500;

    flag_start_OA = false;
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

    if (byte2int(_linebuf, 10) != 0)
        ph2_data.ch.aux7    = byte2int  (_linebuf, 10);

    ph2_data.ch.aux8    = byte2int  (_linebuf, 11);
    ph2_data.u1         = byte2float(_linebuf, 12);
    ph2_data.throttle_in        = byte2float(_linebuf, 13);
    ph2_data.throttle_avg_max   = byte2float(_linebuf, 14);
    ph2_data.thr_hover          = byte2float(_linebuf, 15);
    ph2_data.perr.ez            = byte2float(_linebuf, 16);
    ph2_data.perr.iterm_z       = byte2float(_linebuf, 17);
    ph2_data.perr.dterm_z       = byte2float(_linebuf, 18);


    // assign timestamp to data
    ph2_data.ts_odroid = millis() - _ts_startup;

    // pushing data to ring buffer
    if (ph2_data_q.size() >= MAX_MESSENGER_DATA_QUEUE_SIZE) ph2_data_q.pop_front();
    ph2_data_q.push_back(ph2_data);
}

void messenger::get_tx2_data()
{
    char c;
    char temp_ID;

    _linebufTX2_len = 0;

    if (_sp.DataAvail())
    {
        c = _sp.Getchar();

        if (c == '$')
        {
            temp_ID = _sp.Getchar();

            for (int i=0; i<TX2_MSG_BUF_SIZE; i++)
            {
                _linebufTX2[_linebufTX2_len++] = _sp.Getchar();
            }
        }

    }

    lock_guard<mutex>   lock(_msg_tx2_mtx); // protecting data writing
    // decoding message

    pos_t temp_target;

    if (flag_start_OA)
    {
        temp_target.y = (int) byte2float(_linebufTX2, 0);
        temp_target.z = (int) byte2float(_linebufTX2, 1);

        if ((temp_target.y != prev_target.y) || (temp_target.z != prev_target.z))
        {
            pos_d.y = 1.3*temp_target.y + _ldata.pos.y;
            pos_d.z = 1.3*temp_target.z + _ldata.pos.z;
        }

        prev_target.y = temp_target.y;
        prev_target.z = temp_target.z;
    }


    //cout << "yd " << pos_d.y << " zd " << pos_d.z << " y " << _ldata.pos.y << " z " << _ldata.pos.z << '\n';
}

void messenger::get_pos(pos_data_t ldata)
{
    _ldata = ldata;
}

void messenger::send_pos_data(pos_data_t ldata)
{
    _ldata = ldata;

    vector<unsigned char> msg;
    msg = _pos_msg_encoder();

    for (int i=0; i<msg.size(); i++)
    {
        _sp.putchar(msg[i]);
    }
}


vector<unsigned char> messenger::_pos_msg_encoder()
{
//=================================
//********  MSG FORMAT  ***********
//=================================
//
//  message:
//  $ is_healthy pos.y pos.z nset

    int_num value;

    vector<unsigned char> msg;
    msg.reserve(13);

    msg.push_back('$');

    // is_healthy
    if (_ldata.is_healthy)  value.num = 1;
    else                    value.num = 0;

    for (int i=0; i<4; i++)
    {
        msg.push_back(value.buf[i]);
    }

    // pos y
    value.num = _ldata.pos.y;
    for (int i=0; i<4; i++)
    {
        msg.push_back(value.buf[i]);
    }

    // pos z
    value.num = _ldata.pos.z;
    for (int i=0; i<4; i++)
    {
        msg.push_back(value.buf[i]);
    }

    // nset
    value.num = _ldata.nset;
    for (int i=0; i<4; i++)
    {
        msg.push_back(value.buf[i]);
    }

    // target y
    value.num = pos_d.y;
    for (int i=0; i<4; i++)
    {
        msg.push_back(value.buf[i]);
    }

    // target z
    value.num = pos_d.z;
    for (int i=0; i<4; i++)
    {
        msg.push_back(value.buf[i]);
    }


    //cout << "msg: " << msg << '\n';   // debug

    return msg;
}

void messenger::get_OA_status(bool OA_status)  // obstacle avoidance
{
    flag_start_OA = OA_status;
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
