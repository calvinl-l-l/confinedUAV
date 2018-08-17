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

    // assign timestamp to data
    ph2_data.ts_odroid = millis() - _ts_startup;

    // pushing data to ring buffer
    if (ph2_data_q.size() >= MAX_MESSENGER_DATA_QUEUE_SIZE) ph2_data_q.pop_front();
    ph2_data_q.push_back(ph2_data);
}


void messenger::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;
}
