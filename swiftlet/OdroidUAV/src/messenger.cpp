#include "messenger.h"


messenger::messenger(cSerial sp)
{
    _sp = sp;
}

void messenger::get_data()
{
    char c;
    while (_sp.DataAvail())
    {
        c = _sp.Getchar();

        if (c == '#')   // end bit: start processing data
        {
            // do shit

            _linebuf_len = 0;   // reset
        }
        else
        {
            _linebuf[_linebuf_len++] = c;

            if (_linebuf_len == sizeof(_linebuf)) {_linebuf_len = 0;}
        }
    }

    ph2_data.ts_odroid = millis() - _ts_startup;

    // pushing data to ring buffer
    if (ph2_data_q.size() >= MAX_MESSENGER_DATA_QUEUE_SIZE) ph2_data_q.pop_front();
    ph2_data_q.push_back(ph2_data);
}


void messenger::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;
}
