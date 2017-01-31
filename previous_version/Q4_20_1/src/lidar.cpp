#include "lidar.h"

Hokuyo_lidar::Hokuyo_lidar()
{
    // open ethernet com
    if (!urg.open(connect_address, connect_port, Urg_driver::Ethernet)) cout << "error" << endl;

    // start measurement
    urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
}

void Hokuyo_lidar::read()
{
    // Clear old data vector
    range.clear();
    angle.clear();
    x.clear();
    y.clear();

    if (!urg.get_distance(range, NULL)) cout << "Error reading lidar scan!" << endl;
    // NULL = no geting time stamp

    // converting raw data to cartesian
    for (int i=0; i < 540*2; i++)
    {
        // angle of the range
        double rad = urg.index2rad(i);

        angle.push_back(urg.index2deg(i));
        x.push_back((long) (range[i] * cos(rad)));
        y.push_back((long) (range[i] * sin(rad)));

    }

}  // end of lidar read();

/*
// centroid
void Hokuyo_lidar::get_centroid()
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


    // compute initial ground altitude
    static int atimer = 0;
    static float alt_sum = 0;
    static int GND_alt_saved = 0;
    int n_100ms = 100;
    if (atimer < n_100ms)
    {
        atimer++;
        alt_sum += data->zc;
    }
    else
    {
        if (!GND_alt_saved) data->GND_alt = alt_sum /= n_100ms;

        GND_alt_saved = 1;

    }



} // end of get centroid
} // end centroid

*/

void Hokuyo_lidar::get_altitude()
{
    // two altitude mode:
    // 1. alt relative to top
    // 2. Est. alt from ground

}

void Hokuyo_lidar::wake()
{
    urg.wakeup();
}

void Hokuyo_lidar::sleep()
{
    urg.sleep();
}

void Hokuyo_lidar::close()
{
    urg.close();
}

