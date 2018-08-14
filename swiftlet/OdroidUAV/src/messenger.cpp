#include "messenger.h"


messenger::messenger(cSerial sp)
{
    _sp = sp;
}

void messenger::get_info()
{

}


void messenger::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;
}

void messenger::msg_decoder()
{

}
