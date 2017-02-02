#include "lidar.h"

MedianFilter alt_filter(5,0);


Hokuyo_lidar::Hokuyo_lidar()
{


    // open ethernet com
    if (!urg.open(connect_address, connect_port, Urg_driver::Ethernet)) cout << "error" << endl;

    // start measurement
    urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);

   // do some initial read, get average starting area
   start_area = 0;

   for (int i=0; i < 10; i++)
   {
       read();
       start_area += area;
   }

   start_area /= 10.0f;

}

void Hokuyo_lidar::read()
{
/*
    -----------> +y
    |
    |
    |
    |
    |
    v +z

*/

    // Clear old data vector
    range.clear();
    angle.clear();
    z.clear();
    y.clear();

    if (!urg.get_distance(range, NULL)) cout << "Error reading lidar scan!" << endl;
    // NULL = no geting time stamp

    // converting raw data to cartesian
    int n = 0;
    for (int i=0; i < 540*2; i++)
    {
        // angle of the range
        double rad = urg.index2rad(i);
        angle.push_back(urg.index2deg(i));

        // only store meaningfull data
        if ((range[i] > 0) && (range[i] <= 10000))
        {
            z.push_back((long) (range[i] * cos(rad)));
            y.push_back((long) (range[i] * sin(rad)));
            n++;
        }
    }

    nyz = n;

    get_centroid();

}  // end of lidar read();


// centroid
void Hokuyo_lidar::get_centroid()
{
    float ciy = 0;
    float ciz = 0;
    float cy = 0;
    float cz = 0;
    float temp = 0;
    float Area = 0;
    int n = nyz;


    // compute total area
    for (int i=0;i<=n-1;i++)
    {
        if (i<n-1)
            temp += y[i] * z[i+1] - y[i+1] * z[i];
        else
            temp += y[i] * z[0] - y[0] * z[i];
    }
    float A = fabs(0.5*temp);

    temp = 0; //reset temp variable

    int nTri = n-2;

    // compute centre of area

    for (int i=0;i<=nTri-1;i++)
    {

        temp = y[0]*z[i+1] - y[i+1]*z[0] + y[i+1]*z[i+2] - y[i+2]*z[i+1] + y[i+2]*z[0] - y[0]*z[i+2];

        ciy = y[0] + y[i+1] + y[i+2];
        ciz = z[0] + z[i+1] + z[i+2];

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

    // centroid of the local scan, also equal to the position of the quad relative
    // to local scan
    pos_loc_y = (cy/Area);//(cy/A);
    pos_loc_z = (cz/Area);

    area = Area / (1000*1000);

/*
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
*/

} // end centroid



void Hokuyo_lidar::get_altitude()
{
    // two altitude mode:
    // 1. alt relative to top
    // 2. Est. alt from ground

}


void Hokuyo_lidar::get_dist2wall(float roll)
{
    int nl = 0;
    int nr = 0;
    float i = 80;
    float distR = 0;
    float distL = 0;


    while (i <= 100)
    {
        // distance to right wall
        float temp_R = range[urg.deg2index(-i+roll)] * cos(M_PI*(90-i)/180);

        if ((temp_R != 0) && (temp_R <= 10000))
        {
            nr++;
            distR += temp_R;
        }

        // distance to left wall
        float temp_L = range[urg.deg2index(i+roll)] * cos(M_PI*(90-i)/180);

        if ((temp_L != 0) && (temp_L <= 10000))
        {
            nl++;
            distL += temp_L;
        }

        i += 0.25;
    }

    dist_wallR = distR/nr;
    dist_wallL = distL/nl;
}

int Hokuyo_lidar::lidar_check_boundary()
{
/*
    TO DO:
        extra condition on the huge change in centroid
*/

    float dA = fabs(start_area - area) / start_area*100;

    if (dA > 50)    return 0;
    else            return 1;

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

