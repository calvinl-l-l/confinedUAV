#include "../custom_header/RF_tag.h"

int RF_serial_init()
{
    int fd = open(RF_PORT, O_RDONLY | O_NOCTTY);

    if (fd < 0) {perror(RF_PORT); exit(-1);}

    struct termios options;

    bzero(&options, sizeof(options));
    options.c_cflag = RF_BAUDRATE | CS8 | CLOCAL | CREAD | IGNPAR;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}
