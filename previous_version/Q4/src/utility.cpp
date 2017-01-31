#include "utility.h"


void delay(int ms)
{
    usleep(1000 * ms);
}


long val_remap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
