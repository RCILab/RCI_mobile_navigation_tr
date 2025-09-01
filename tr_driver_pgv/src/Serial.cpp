#include "Serial.h"
#include <iostream>
#include <fstream>

using namespace std;

int open_serial(char *dev_name, int baud, int vtime, int vmin)
{
    int fd;
    struct termios newtio;

    //open serial port
    fd = open(dev_name, O_RDWR);
    //cout << "hahah" << endl;
    if(fd < 0)
    {
        //fail open
    	cout << "fail" << endl;
        return -1;
    }

    // port configure
    memset(&newtio, 0, sizeof(newtio));
    
    if(tcgetattr(fd,&newtio) != 0)
    	printf("Error %i from tcgetattr: %s\n",errno,strerror(errno));
    
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag |= CS7;
    newtio.c_cflag &= ~CRTSCTS;
    newtio.c_cflag |= CREAD | CLOCAL;
    newtio.c_lflag &= ~ICANON;
    newtio.c_lflag &= ~ECHO;
    newtio.c_lflag &= ~ECHOE;
    newtio.c_lflag &= ~ECHONL;
    newtio.c_lflag &= ~ISIG;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
    newtio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    newtio.c_oflag &= ~OPOST;
    newtio.c_oflag &= ~ONLCR;

    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);

    
    newtio.c_cc[VTIME] = vtime;     // timeout 0.1s
    newtio.c_cc[VMIN] = vmin;       // wait

    if(tcsetattr(fd, TCSANOW, &newtio) != 0)
	printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));

    return fd;
}
void close_serial(int fd)
{
    close(fd);
}




