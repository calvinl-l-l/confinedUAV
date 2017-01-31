#ifndef RF_TAG_H
#define RF_TAG_H

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include "../mavlink_lib/serial_port.h"

#define RF_PORT         "/dev/ttyACM1"
#define RF_BAUDRATE     B9600

int RF_serial_init();





#endif // RF_TAG_H
