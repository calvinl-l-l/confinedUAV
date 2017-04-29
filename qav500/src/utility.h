#include <wiringSerial.h>
#include <iostream>
#include <unistd.h>
#include <fstream>



long val_remap(long x, long in_min, long in_max, long out_min, long out_max);
void signal_LED(int fd, int boundary, int mode);
