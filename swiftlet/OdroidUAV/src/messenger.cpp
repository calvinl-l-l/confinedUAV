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

    // decoding message
    int r           = (int) (_linebuf[0]<<24|_linebuf[1]<<16|_linebuf[2]<<8|_linebuf[3]);
    int p           = (int) (_linebuf[4]<<24|_linebuf[5]<<16|_linebuf[6]<<8|_linebuf[7]);
    int y           = (int) (_linebuf[8]<<24|_linebuf[9]<<16|_linebuf[10]<<8|_linebuf[11]);
    ph2_data.ts_PH2 = (int) (_linebuf[12]<<24|_linebuf[13]<<16|_linebuf[14]<<8|_linebuf[15]);

    ph2_data.roll   = (float) r/1000.0f;
    ph2_data.pitch  = (float) p/1000.0f;
    ph2_data.yaw    = (float) y/1000.0f;

    ph2_data.target_climb_rate = (int) (_linebuf[16]<<24|_linebuf[17]<<16|_linebuf[18]<<8|_linebuf[19]);
    int temp_alt_target        = (int) (_linebuf[20]<<24|_linebuf[21]<<16|_linebuf[22]<<8|_linebuf[23]);
    ph2_data.alt_target = (float) temp_alt_target/1000.0f;

    // assign timestamp to data
    ph2_data.ts_odroid = millis() - _ts_startup;

    //printf("roll %f\n", ph2_data.roll);

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
//  $ is_healthy pos_y pos_z alt #
//  1       1      6     6    6  1 bytes, total = 18 bytes
    string msg = "";

    msg = "$" + to_string(_ldata.is_healthy);
    msg += int2str_5digits(_ldata.pos_y);
    msg += int2str_5digits(_ldata.pos_z);
    msg += int2str_5digits(_ldata.alt);
    msg += "#";

    return msg;
}



void messenger::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;
}
