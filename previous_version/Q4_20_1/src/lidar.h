#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "urg_cpp/Lidar.h"
#include "urg_cpp/Urg_driver.h"


using namespace std;
using namespace qrk;

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;

class Hokuyo_lidar
{
    public:
        Hokuyo_lidar();
        void read();
        void get_centroid();
        void get_altitude();
        void sleep();
        void wake();
        void close();

        vector<long> range;
        vector<double> angle;
        vector<long> x;
        vector<long> y;

    private:
        Urg_driver urg;

};
