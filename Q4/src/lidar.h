#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "urg_cpp/Lidar.h"
#include "urg_cpp/Urg_driver.h"
#include "MedianFilter.h"

using namespace std;
using namespace qrk;

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;


class Hokuyo_lidar
{
    public:

        vector<long> range;
        vector<double> angle;
        vector<long> y;
        vector<long> z;
        int nyz;
        float pos_loc_y;
        float pos_loc_z;
        float area;
        float start_area;
        int alt_floor;

        Hokuyo_lidar();
        void read();
        void get_centroid();
        int lidar_check_boundary();
        void get_altitude();
        void sleep();
        void wake();
        void close();

    private:
        Urg_driver urg;



};
