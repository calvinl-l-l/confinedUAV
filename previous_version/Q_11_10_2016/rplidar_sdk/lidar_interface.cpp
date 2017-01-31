/*
    note:
    90 degree -> left
    270 degree -> right
    180 degree -> top
*/


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "lidar_interface.h"


// *** DEGBUG PRINT OUT ***
//#define PRINT_RANGE_RAW
//#define PRINT_YZ


bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d, err code: %d\n", healthinfo.status, healthinfo.error_code);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            printf("Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        printf("Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}


void getLidarInfo(RPlidarDriver * drv)
{
    u_result op_result;
    rplidar_response_device_info_t deviceinfo;

    op_result = drv->getDeviceInfo(deviceinfo);
    //freq_result = drv->getFrequency()
    if (IS_OK(op_result))
    {
        printf("model %d, firmware_version %d\n", deviceinfo.model, deviceinfo.serialnum);
        if (drv->isConnected())
        {
            printf("Lidar is connected\n");
        }
        else
        {
            printf("Lidar is not even connected!!!\n");
        }

    }
    else
    {
        printf("Error getting device info - code: %x\n", op_result);
    }

}

//RPlidarDriver *drv_ = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
void lidar_init(RPlidarDriver * drv, char* serial_port, _u32 baudrate)
{

    // make connection...
    if (IS_FAIL(drv->connect(serial_port, baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , serial_port);
        exit(-2);
    }



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        exit(-2);
    }


    // start scan...

    drv->startScan();

}

void lidar_read(RPlidarDriver * drv, lidar_data* data, MedianFilter* ldata_yc, MedianFilter* ldata_zc)
{

    rplidar_response_measurement_node_t nodes[360*2];
    size_t   count = _countof(nodes);

    u_result op_result = drv->grabScanData(nodes, count);

    if (IS_OK(op_result))
    {
        data->prev_nyz = data->nyz;

        drv->ascendScanData(nodes, count);

        data->nyz = 0;

        for (int pos = 0; pos < (int)count ; ++pos)
        {
            //display range data
            #ifdef PRINT_RANGE_RAW
            printf("%s theta: %03.2f Dist: %08.2f Q: %d, count: %d\n",
                (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
                (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                 nodes[pos].distance_q2/4.0f,
                 nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT, pos);
            #endif

            data->dist[pos]  = nodes[pos].distance_q2/4.0f;
            data->angle[pos] = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
            data->nraw = (int) count;

            //polar to cartesian
            if ((data->dist[pos] != 0)&&(data->dist[pos]<=5000))
            {
                data->y[data->nyz] = data->dist[pos] * sin(data->angle[pos]*M_PI/180);
                data->z[data->nyz] = data->dist[pos] * cos(data->angle[pos]*M_PI/180);

                data->yz_angle[data->nyz] = data->angle[pos];

                data->nyz++;
            }




            #ifdef PRINT_YZ
                printf("y %4.2f    z     %4.2f   n   %d  yc %4.2f  zc %4.2f  %s\n", data->y[pos], data->z[pos], data->nyz, data->yc, data->zc, (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ");
            #endif
        }
        data->nyz--;

        getCentroid(data, ldata_yc, ldata_zc);
    }
    else
    {
        printf("error reading lidar data, code: %x\n", op_result);

    }





}

void getCentroid(lidar_data* data, MedianFilter* ldata_yc, MedianFilter* ldata_zc)
{
    float ciy = 0;
    float ciz = 0;
    float cy = 0;
    float cz = 0;
    float temp = 0;
    float Area = 0;
    int n = data->nyz;


    // compute total area
    for (int i=0;i<=n-1;i++)
    {
        if (i<n-1)
            temp += data->y[i]*data->z[i+1] - data->y[i+1]*data->z[i];
        else
            temp += data->y[i]*data->z[0] - data->y[0]*data->z[i];
    }
    float A = fabs(0.5*temp);

    temp = 0; //reset temp variable

    int nTri = n-2;

    // compute centre of area

    for (int i=0;i<=nTri-1;i++)
    {

        temp = data->y[0]*data->z[i+1] - data->y[i+1]*data->z[0] + data->y[i+1]*data->z[i+2] - data->y[i+2]*data->z[i+1] + data->y[i+2]*data->z[0] - data->y[0]*data->z[i+2];

        ciy = data->y[0] + data->y[i+1] + data->y[i+2];
        ciz = data->z[0] + data->z[i+1] + data->z[i+2];

        ciy /= 3;
        ciz /= 3;

        float iTriA = fabs(0.5*temp);

        cy += ciy*iTriA;
        cz += ciz*iTriA;

        temp = 0;
        ciy = 0;
        ciz = 0;

        Area += iTriA;
    }

    data->prev_yc = data->yc;
    data->prev_zc = data->zc;


    float predict_yc = cy/A - data->prev_yc + cy/A;
    float predict_zc = cz/A - data->prev_yc + cz/A;

    // predicted centroid
    //data->yc = ldata_yc->in(cy/A);//(cy/A);
    //data->zc = ldata_zc->in(cz/A);//(cz/A);

    // normal centroid
    data->yc = (cy/Area);//(cy/A);
    data->zc = (cz/Area);


    data->area = Area / (1000*1000);
    //cout << "A " << A << " Area " << Area << endl;
    //if (abs(data->yc-data->prev_yc)> 300)   data->yc = data->prev_yc;
    //if (abs(data->zc-data->prev_zc)> 300)   data->zc = data->prev_zc;
} // end of get centroid


// temporary positioning: get alt
// need to calculate for the pitch angle as well
void lidar_alt(RPlidarDriver * drv, float roll, float pitch, lidar_data* data, int lidar_RNG)
{

    //data->alt = data->dist[0];
    int n = 0;
    int alt = 0;
    int ndata = data->nraw;

    for (int i=0;i<=lidar_RNG;i++)
    {
        if (data->dist[i] != 0)
        {
            n++;
            alt += data->dist[i]*cos(data->angle[i]*M_PI/180);
        }
        if (data->dist[ndata-i-1] != 0)
        {
            n++;
            alt += data->dist[ndata-i-1]*cos(2*M_PI-data->angle[ndata-i-1]*M_PI/180);
        }
    }

    alt *= cos(roll*M_PI/180);
    alt *= cos(pitch*M_PI/180);

    if (n>0)
    {
        data->alt = alt/n;
    }
    else
    {
        data->alt = data->alt;
    }


    //printf("al = %.2f  centre alt = %.2f  ", data->alt, data->dist[0]);


}


// checking if lidar scan make sense. If out of boundary then return false. TRUE = pass
int lidar_check_boundary(lidar_data data)
{
/*
    TO DO:
        extra condition on the huge change in centroid
*/

    float dA = fabs(data.start_area - data.area)/data.start_area*100;

    if (dA > 50)    return 0;
    else            return 1;



}

//=========================== socket com helper functions ====================================================
void prepare_PID_msg(float kp, float ki, float kd, char* msg_header)
{

    char header[buffer_size_PID_msg];
    int n = 0;
    unsigned long long spacer = 0;
    int space = 0;


    snprintf(header, buffer_size_PID_msg, "H %.3f %.3f %.3f ", kp, ki, kd);
    n = nChar(header);

    // calculating spacer value
    space = buffer_size_PID_msg - n - 2;


    if (space >= 2)    spacer = 10;


    for (int i=0;i<space-2;i++)
    {
        spacer *= 10;

    }


    // building header
    // s nraw y z roll yaw time mode
    snprintf(header + n, buffer_size_PID_msg, "%lld h", spacer);

    // transfering data to buffer
    for (int i=0;i<buffer_size_PID_msg;i++)
    {
        msg_header[i] = header[i];
    }

    //cout << "header " << header << endl;

}


void prepare_sock_action_header(string action, string param, char* msg_header)
{

    char header[buffer_size_action_header];
    int n = 0;
    unsigned long long spacer = 0;
    int space = 0;

    //snprintf(header, buffer_size_action_header, "$ %c %c ", &action[0], &param[0]);

    strcat(header, "$ ");
    strcat(header, action.c_str());
    strcat(header, " ");
    strcat(header, param.c_str());
    strcat(header, " ");

    n = nChar(header);

    // calculating spacer value
    space = buffer_size_action_header - n - 2;


    if (space >= 2)    spacer = 10;


    for (int i=0;i<space-2;i++)
    {
        spacer *= 10;

    }


    // building header
    // s nraw y z roll yaw time mode
    snprintf(header + n, buffer_size_action_header, "%lld !", spacer);

    // transfering data to buffer
    for (int i=0;i<buffer_size_action_header;i++)
    {
        msg_header[i] = header[i];
    }
    //cout << "Action header: " << header << " n " << n << " space " << space << " total " << nChar(header) << endl;

}


// lidar h
void prepare_sock_msg_header(lidar_data ldata, pixhawk_data pixdata, double time_stamp, int mode, int RF, char* msg_header)
{

    char header[buffer_size_header];
    int n = 0;
    unsigned long long spacer = 0;
    int space = 0;

    if (ldata.yc==NAN) ldata.yc = 0;
    if (ldata.zc==NAN) ldata.zc = 0;

    snprintf(header, buffer_size_header, "H %d %.2f %.2f %.2f %.2f %.1f %d %.2f %.2f ", ldata.nraw, ldata.yc, ldata.zc, pixdata.roll, pixdata.yaw, time_stamp, mode, ldata.area, (float) RF/1000.0);
    n = nChar(header);

    // calculating spacer value
    space = buffer_size_header - n - 2;


    if (space >= 2)    spacer = 10;


    for (int i=0;i<space-2;i++)
    {
        spacer *= 10;

    }


    // building header
    // s nraw y z roll yaw time mode
    snprintf(header + n, buffer_size_header, "%lld h", spacer);

    // transfering data to buffer
    for (int i=0;i<buffer_size_header;i++)
    {
        msg_header[i] = header[i];
    }

    //cout << "header " << header << endl;

}

// lidar h
void prepare_sock_msg_data(lidar_data ldata, int idx, int data_end, char* msg_data)
{
    char data[buffer_size_data];
    int n = 0;
    int space = 0;
    unsigned long long spacer = 0;

    snprintf(data, buffer_size_data, "D %.5f %.5f %d ", ldata.angle[idx], ldata.dist[idx], data_end);
    n = nChar(data);


    // calculating spacer value
    space = buffer_size_data - n - 2;

    if (space >= 2)    spacer = 10;

    for (int i=0;i<space-2;i++)
    {
        spacer *= 10;
    }


    // building header
    snprintf(data + n, buffer_size_data, "%lld d", spacer);


    // transfering data to buffer
    for (int i=0;i<buffer_size_data;i++)
    {
        msg_data[i] = data[i];
    }
    //cout << "data -> " << data << endl;
}

// lidar h
int nChar(char* c)
{
    return string(c).length();
}


