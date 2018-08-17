#include "lidar.h"


Hokuyo_lidar::Hokuyo_lidar()
{
    // open ethernet com
    if (!_urg.open(connect_address, connect_port, Urg_driver::Ethernet))
    {
        cout << "Error: unable to connect to lidar!!" << endl;
        exit(EXIT_FAILURE);
    }

    // start measurement
    _urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);

    // init flag
    flag.init_startup_block = true;
    flag.lidar_error = false;
    flag.outof_boundary = false;
    flag.printed_alt_mode = false;
    flag.alt = ROOF;
    _tunnel_height = 0;
    
    _init_alt_type();   // TODO: test open space init for alt type


}

void Hokuyo_lidar::read()
{
    // Clear old data vector
    ldata.range.clear();
    ldata.angle.clear();
    ldata.pc_z.clear();
    ldata.pc_y.clear();

    if (!_urg.get_distance(ldata.range, &ldata.ts_lidar))
    {
      flag.lidar_error  = true;
      cout << "Error reading lidar scan!" << endl;
    }
    else
    {
      flag.lidar_error = false;
    }

    ldata.ts_odroid = millis() - _ts_startup;    // NULL = no geting time stamp
}

void Hokuyo_lidar::pos_update()
{
    int n = 0;
    double roll = _ph2_data.roll;   // in rad, roll angle from PH2

    for (int i=0; i < 540*2; i++)
    {
        // angle of the range
        double rad = _urg.index2rad(i);
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

    // TODO: add proper rotation matrix later
    // apply roll rotation
    for (int i=0; i < ldata.nyz; i++)
    {
        double y_temp = cos(roll) * ldata.pc_y[i] - sin(roll) * ldata.pc_z[i];
        double z_temp = sin(roll) * ldata.pc_y[i] + cos(roll) * ldata.pc_z[i];

        ldata.pc_y[i] = y_temp;
        ldata.pc_z[i] = z_temp;
    }

    // localisation algorithm
    calc_alt();

    // TODO --> here


    // pushing lidar data to the queue
    if (ldata_q.size() >= MAX_LDATA_QUEUE_SIZE) ldata_q.pop_front();    // limit memory usage
    ldata_q.push_back(ldata);
}


void Hokuyo_lidar::calc_alt()
{
    // angle sign use right hand rule
    double pitch = _ph2_data.pitch;     // TODO: add proper rotation matrix later
    double roll  = _ph2_data.roll;

    unsigned int temp = 0;
    int n = 0;

    switch(flag.alt)
    {
        case FLOOR:
            if (roll > ALT_FLOOR_ANGLE_RANGE/2)
            // no -ve side
            {
                for (float i=135; i >= 135 - ALT_FLOOR_ANGLE_RANGE; i-=0.25)
                {
                    int idx = _urg.deg2index(i);
                    temp += ldata.range[idx] * cos(deg2r(i)+roll);
                    n++;
                }
            }
            else if (roll < -ALT_FLOOR_ANGLE_RANGE/2)
            // no +ve side
            {
                for (float i=-135; i <= -135 + ALT_FLOOR_ANGLE_RANGE; i+=0.25)
                {
                    int idx = _urg.deg2index(i);
                    temp += ldata.range[idx] * cos(deg2r(i)+roll);
                    n++;
                }
            }
            else
            {
                // +ve side
                for (float i=135; i >= 135 - ALT_FLOOR_ANGLE_RANGE/2 - fabs(r2deg(roll)); i-=0.25)
                {
                    int idx = _urg.deg2index(i);
                    temp += fabs(ldata.range[idx] * cos(deg2r(i)+roll));
                    n++;
                }

                // -ve side
                for (float i=-135; i <= -135 + ALT_FLOOR_ANGLE_RANGE/2 + fabs(r2deg(roll)); i+=0.25)
                {
                    int idx = _urg.deg2index(i);
                    temp += fabs(ldata.range[idx] * cos(deg2r(i)+roll));
                    n++;
                }

            }

            break;

        case ROOF:
            for (float i=-ALT_ROOF_ANGLE_RANGE/2; i<=ALT_ROOF_ANGLE_RANGE/2; i+=0.25)
            {
                int idx = _urg.rad2index(deg2r(i)-roll);
                temp += fabs(ldata.range[idx] * cos(deg2r(i)));
                n++;
            }
            break;

        case BOTH:
            cout << "Not available yet!!\n";
            break;
    }

    ldata.alt = (unsigned int) temp/n;

    if (flag.alt == ROOF)   ldata.alt = abs(_tunnel_height - ldata.alt);

    printf("alt: %d\n", ldata.alt);
}

vector<int> Hokuyo_lidar::_pt2spectrum(vector<double> point)
{
    _specY_src.clear();
    _specZ_src.clear();

}

void Hokuyo_lidar::_scan_matching_sptm()
{

  // max = *max_element(vector.begin(), vector.end());
}

// to be removed
void Hokuyo_lidar::_get_centroid2()
{
    double A = 0;
    double cy = 0;
    double cz = 0;

    for (int i=0; i <= ldata.nyz; i++)
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

void Hokuyo_lidar::print_alt_type()
{
    if (!flag.printed_alt_mode)
    {
        string a;

        switch (flag.alt)
        {
            case 0:  a = "floor";
                break;
            case 1:  a = "roof";
                break;
            case 2:  a = "both";
                break;
        }

        cout << "\n\n";
        cout << "========================================\n";
        cout << "   altitude type is: " << a << '\n';
        cout << "========================================\n\n";

        flag.printed_alt_mode = true; // reset flag
    }
}

void Hokuyo_lidar::_init_alt_type()
{
    cout << "Computing altitude type . . .\n";

    int d0;
    int d45;
    int d_45;

    deque<lidar_data_t> temp_q;
    lidar_data_t temp;

    for (int i=0; i < 20; i++)
    {
        read();
        if (i >= 10)    // discard the first 10 scan just incase
        {
            temp_q.push_back(ldata);
        }
    }

    // averge the 10 scan
    for (int i=0; i < 10; i++)
    {
        // picking -45 0 45 degree to check if a roof is in range
        temp = temp_q.front();
        temp_q.pop_front();

        d0 += temp.range[_urg.deg2index(0)];
        d45 += temp.range[_urg.deg2index(45)];
        d_45 += temp.range[_urg.deg2index(-45)];
    }

    d0 /= 10;
    d45 /= 10;
    d_45 /= 10;

    d45 /= sin(deg2r(45));
    d_45 /= sin(deg2r(45));

    unsigned int score = 0;
    if (d0 > ROOF_THRESHOLD)    score++;
    if (d45 > ROOF_THRESHOLD)    score++;
    if (d_45 > ROOF_THRESHOLD)    score++;

    if (score >= 2) set_alt_type(FLOOR);
    else            set_alt_type(ROOF);

    flag.init_startup_block = false;
    print_alt_type();   // done
}

void Hokuyo_lidar::set_alt_type(lidar_alt_type dir)
{
    flag.alt = dir;

    if (dir == ROOF)
    {
        _tunnel_height = 0; // reset tunnel height
        calc_alt();

        _tunnel_height = abs(ldata.alt);
        cout << "height " << _tunnel_height << endl;
    }
}

void Hokuyo_lidar::wake()
{
    _urg.wakeup();
}

void Hokuyo_lidar::sleep()
{
    _urg.sleep();
}

void Hokuyo_lidar::close()
{
    _urg.close();
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

double Hokuyo_lidar::r2deg(double radian)
{
    return radian*M_PI/180;
}

/*
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


bool Hokuyo_lidar::_lidar_check_flag.outof_boundary()
{

    //TODO:
        extra condition on the huge change in centroid


    // trigger flag (flag=1) when out of boundary

    return false;
}


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
