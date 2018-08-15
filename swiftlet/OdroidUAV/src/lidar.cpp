#include "lidar.h"


Hokuyo_lidar::Hokuyo_lidar()
{


    // open ethernet com
    if (!urg.open(connect_address, connect_port, Urg_driver::Ethernet)) cout << "Error: unable to connect to lidar!!" << endl;

    // start measurement
    urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);

}

void Hokuyo_lidar::read()
{
    // Clear old data vector
    ldata.range.clear();
    ldata.angle.clear();
    ldata.pc_z.clear();
    ldata.pc_y.clear();

    if (!urg.get_distance(ldata.range, &ldata.ts_lidar))
    {
      flag_lidar_error  = true;
      cout << "Error reading lidar scan!" << endl;
    }
    else
    {
      flag_lidar_error = false;
    }

    ldata.ts_odroid = millis() - _ts_startup;    // NULL = no geting time stamp
}  // end of lidar read();

void Hokuyo_lidar::pos_update()
{
    int n = 0;

    for (int i=0; i < 540*2; i++)
    {
        // angle of the range
        double rad = urg.index2rad(i);
        ldata.angle.push_back(rad);

        // from Polar to Cartesian
        if (!(fabs(ldata.angle[i]) > deg2r(80) && fabs(ldata.angle[i]) < deg2r(90)) && // position of props
            !(fabs(ldata.angle[i]) > deg2r(130)))  // end 5 degree on both side
        {
            // for Swiftlet frame, skip 81.5 to 89.75 degree
            ldata.pc_z.push_back((double) (ldata.range[i] * cos(rad)));
            ldata.pc_y.push_back((double) (ldata.range[i] * sin(rad)));
            n++;
        }
    }

    ldata.nyz = n;

    _data_loss = (float) n/1080;  // calc data loss

// TODO
/*
    for (int i=0; i < ldata.nyz; i++)
    {
        double y_temp = cos(roll*M_PI/180.0f) * ldata.pc_y[i] - sin(roll*M_PI/180.0f) * ldata.pc_z[i];
        double z_temp = sin(roll*M_PI/180.0f) * ldata.pc_y[i] + cos(roll*M_PI/180.0f) * ldata.pc_z[i];

        ldata.pc_y[i] = y_temp;
        ldata.pc_z[i] = z_temp;
    }
*/

    // pushing lidar data to the queue
    if (ldata_q.size() >= MAX_LDATA_QUEUE_SIZE) ldata_q.pop_front();    // limit memory usage
    ldata_q.push_back(ldata);
}

// to be removed
void Hokuyo_lidar::_get_centroid2()
{
    double A = 0;
    double cy = 0;
    double cz = 0;

    for (int i=yz_start_pt; i <= yz_end_pt; i++)
    {
        A +=  (ldata.pc_y[i] * ldata.pc_z[i+1]) - (ldata.pc_y[i+1] * ldata.pc_z[i]);

        cy += (ldata.pc_y[i] + ldata.pc_y[i+1]) * (ldata.pc_y[i] * ldata.pc_z[i+1] - ldata.pc_y[i+1] * ldata.pc_z[i]);
        cz += (ldata.pc_z[i] + ldata.pc_z[i+1]) * (ldata.pc_y[i] * ldata.pc_z[i+1] - ldata.pc_y[i+1] * ldata.pc_z[i]);
        //cout << cy << endl;
    }

    A /= 2.0f;


    ldata.pos_y = cy/(6.0f * A);
    ldata.pos_z = cz/(6.0f * A);

    //cout << pos_loc_y2 << endl;

    A /= -(1000.0f*1000.0f);
    ldata.area = A;
    //cout << pos_loc_y2 << ' ' <<  A << endl;
}


// to be removed/updated
void Hokuyo_lidar::_get_symmetry_pt()
{
    yz_start_pt = 0;
    yz_end_pt = ldata.nyz - 1;

    // check if points are connected
    // left side
    for (int i = 20; i > 0; i--)
    {
        // 86mm is threshold, max vertical distance between points, assuming 10m range
        if (fabs(ldata.pc_z[i] - ldata.pc_z[i-1]) <= 86)
        {
            yz_start_pt = i - 1;
        }
        else    {break;}
    }

    // right side
    for (int i = ldata.nyz-21; i < ldata.nyz - 1; i++)
    {
        // 86mm is threshold, max vertical distance between points, assuming 10m range
        if (fabs(ldata.pc_z[i] - ldata.pc_z[i+1]) <= 86)
        {
            yz_end_pt = i + 1;
        }
        else    {break;}
    }

    // compare which side is lower
    if (ldata.pc_z[yz_start_pt] > ldata.pc_z[yz_end_pt])
    {
        for (int i=ldata.nyz-1; i > (int) ldata.nyz/2; i--)
        {
            if ((ldata.pc_z[i] >= ldata.pc_z[yz_start_pt]) && (ldata.pc_z[i-1] >= ldata.pc_z[yz_start_pt]))
            {
                yz_end_pt = i;
                break;
            }
        }
    }
    else if (ldata.pc_z[yz_start_pt] < ldata.pc_z[yz_end_pt])
    {
        for (int i=0; i < (int) ldata.nyz/2; i++)
        {
            if ((ldata.pc_z[i] >= ldata.pc_z[yz_end_pt]) && (ldata.pc_z[i+1] >= ldata.pc_z[yz_end_pt]))
            {
                yz_start_pt = i;

  break;
            }
        }
    }
}


bool Hokuyo_lidar::_lidar_check_outof_boundary()
{
/*
    TO DO:
        extra condition on the huge change in centroid
*/

    // trigger flag (flag=1) when out of boundary

    return false;
}

/*
vector<int> Hokuyo_lidar::lonely_pts_detector()
{
  // output
  vector<int> I;

  // finding quantiles
  int m = median(range);

  vector<long> temp = range;

  sort(temp.begin(), temp.end());

  int m_idx = 0;

  for (int i=0;i<range.size();i++)
  {
    if (abs(temp[i]-m) < 10)  m_idx = i;// 10 is threshold
  }

  vector<long> v2(temp.begin(), temp.begin() + m_idx);
  vector<long> v3(temp.begin() + m_idx + 1, temp.end());

  int q2 = median(v2);
  int q3 = median(v3);
  int IQR = q3 - q2;

  int rmax = 1.5 * IQR + q3;
  int rmin = q2 - 1.5 * IQR;

  // removing the lonely points
  int c = 0;

  for (int i=0;i<range.size();i++)
  {
    if (ldata.range[i] > rmax || ldata.range[i] < rmin) I.push_back(i);
  }

  return I;
}
*/

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

void Hokuyo_lidar::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;

}

void Hokuyo_lidar::get_PH2_data(PH2_data_t data)
{
    _ph2_data = data;
}

double Hokuyo_lidar::deg2r(double degree)
{
    return degree*M_PI/180;
}

/*
// old centroid - to be removed
void Hokuyo_lidar::_get_centroid1()
{
    float ciy = 0;
    float ciz = 0;
    float cy = 0;
    float cz = 0;
    float temp = 0;
    float Area = 0;
    int n = ldata.nyz;


    // compute total area
    for (int i=0;i<=n-1;i++)
    {
        if (i<n-1)
            temp += ldata.pc_y[i] * ldata.pc_z[i+1] - ldata.pc_y[i+1] * ldata.pc_z[i];
        else
            temp += ldata.pc_y[i] * ldata.pc_z[0] - ldata.pc_y[0] * ldata.pc_z[i];
    }
    float A = fabs(0.5*temp);

    temp = 0; //reset temp variable

    int nTri = n-2;

    // compute centre of area

    for (int i=0;i<=nTri-1;i++)
    {

        temp = ldata.pc_y[0]*ldata.pc_z[i+1] - ldata.pc_y[i+1]*ldata.pc_z[0] + ldata.pc_y[i+1]*ldata.pc_z[i+2] - ldata.pc_y[i+2]*ldata.pc_z[i+1] + ldata.pc_y[i+2]*ldata.pc_z[0] - ldata.pc_y[0]*ldata.pc_z[i+2];

        ciy = ldata.pc_y[0] + ldata.pc_y[i+1] + ldata.pc_y[i+2];
        ciz = ldata.pc_z[0] + ldata.pc_z[i+1] + ldata.pc_z[i+2];

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
    ldata.pos_y = (cy/Area);//(cy/A);
    ldata.pos_z = (cz/Area);

    ldata.area = Area / (1000*1000);



} // end centroid
*/
