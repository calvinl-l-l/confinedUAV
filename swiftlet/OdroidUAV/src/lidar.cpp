#include "lidar.h"

Hokuyo_lidar::Hokuyo_lidar() {}

void Hokuyo_lidar::lidar_init()
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
    _max_scan_range = 6*1000;   // default 6m scan range
    _init_alt_type();   // TODO: test open space init for alt type


}

void Hokuyo_lidar::read_scan()
{
    // Clear old data vector
    data.range.clear();
    data.angle.clear();
    data.pc_z.clear();
    data.pc_y.clear();

    if (!_urg.get_distance(data.range, &data.ts_lidar))
    {
      flag.lidar_error  = true;
      data.is_healthy  = false;
      cout << "Error reading lidar scan!" << endl;
    }
    else
    {
      flag.lidar_error = false;
      data.is_healthy = true;
    }

    data.ts_odroid = millis() - _ts_startup;    // NULL = no geting time stamp
}

void Hokuyo_lidar::update_pc()
{
    int n = 0;
    double roll  = -_ph2_data.roll;     // in rad, roll angle from PH2
    double pitch = -_ph2_data.pitch;    // -ve due to lidar orientation

    for (int i=0; i < 540*2; i++)
    {
        // angle of the range
        double rad = _urg.index2rad(i);
        data.angle.push_back(rad);

        // from Polar to Cartesian
        if (!(fabs(data.angle[i]) > deg2r(80) && fabs(data.angle[i]) < deg2r(90)) && // position of props
            !(fabs(data.angle[i]) > deg2r(130)) &&  // end 5 degree on both side
            data.range[i] <= _max_scan_range)
        {
            // for Swiftlet frame, skip 81.5 to 89.75 degree
            data.pc_z.push_back((int) (data.range[i] * cos(rad)) - _offset_z);
            data.pc_y.push_back((int) (data.range[i] * sin(rad)));
            n++;
        }
    }

    data.nyz = n;

    _data_loss = (float) n/1000*100;  // calc data loss

    // applying transformation matrix
    for (int i=0; i < data.nyz; i++)
    {
        double y_temp = cos(roll)*data.pc_y[i] - cos(pitch)*sin(roll)*data.pc_z[i] + _offset_x*sin(roll)*sin(pitch);
        double z_temp = sin(roll)*data.pc_y[i] + cos(pitch)*cos(roll)*data.pc_z[i] - _offset_x*cos(roll)*sin(pitch);

        data.pc_y[i] = y_temp;
        data.pc_z[i] = z_temp;
    }

}


void Hokuyo_lidar::calc_alt()
{
    // angle sign use right hand rule
    double pitch = -_ph2_data.pitch;    // TODO: add proper rotation matrix later
    double roll  = -_ph2_data.roll;     // -ve due to lidar orientation

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
                    temp += data.range[idx] * cos(deg2r(i)+roll);
                    n++;
                }
            }
            else if (roll < -ALT_FLOOR_ANGLE_RANGE/2)
            // no +ve side
            {
                for (float i=-135; i <= -135 + ALT_FLOOR_ANGLE_RANGE; i+=0.25)
                {
                    int idx = _urg.deg2index(i);
                    temp += data.range[idx] * cos(deg2r(i)+roll);
                    n++;
                }
            }
            else
            {
                // +ve side
                for (float i=135; i >= 135 - ALT_FLOOR_ANGLE_RANGE/2 - fabs(r2deg(roll)); i-=0.25)
                {
                    int idx = _urg.deg2index(i);
                    temp += fabs(data.range[idx] * cos(deg2r(i)+roll));
                    n++;
                }

                // -ve side
                for (float i=-135; i <= -135 + ALT_FLOOR_ANGLE_RANGE/2 + fabs(r2deg(roll)); i+=0.25)
                {
                    int idx = _urg.deg2index(i);
                    temp += fabs(data.range[idx] * cos(deg2r(i)+roll));
                    n++;
                }

            }

            break;

        case ROOF:
            for (float i=-ALT_ROOF_ANGLE_RANGE/2; i<=ALT_ROOF_ANGLE_RANGE/2; i+=0.25)
            {
                int idx = _urg.rad2index(deg2r(i)-roll);
                temp += fabs(data.range[idx] * cos(deg2r(i)));
                n++;
            }
            break;

        case BOTH:
            cout << "Not available yet!!\n";
            break;
    }


    data.alt = (unsigned int) temp/n;
    data.alt = (data.alt-_offset_z)*cos(pitch) - _offset_x*sin(pitch);

    if (flag.alt == ROOF)   data.alt = abs(_tunnel_height - data.alt);

    //printf("alt: %d\n", data.alt);   // debug
}


// to be removed
void Hokuyo_lidar::_get_centroid2()
{
    double A = 0;
    double cy = 0;
    double cz = 0;

    for (int i=0; i <= data.nyz; i++)
    {
        A +=  (data.pc_y[i] * data.pc_z[i+1]) - (data.pc_y[i+1] * data.pc_z[i]);

        cy += (data.pc_y[i] + data.pc_y[i+1]) * (data.pc_y[i] * data.pc_z[i+1] - data.pc_y[i+1] * data.pc_z[i]);
        cz += (data.pc_z[i] + data.pc_z[i+1]) * (data.pc_y[i] * data.pc_z[i+1] - data.pc_y[i+1] * data.pc_z[i]);
        //cout << cy << endl;
    }

    A /= 2.0f;


    data.pos.y = cy/(6.0f * A);
    data.pos.z = cz/(6.0f * A);

    //cout << pos_loc_y2 << endl;

    A /= -(1000.0f*1000.0f);
    data.area = A;
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

void Hokuyo_lidar::_save_ref_scan()
{
    read_scan();
    _data_ref = data;
}

void Hokuyo_lidar::_init_alt_type()
{
    cout << "Computing altitude type . . .\n";

    int d0;
    int d45;
    int d_45;

    deque<pos_data_t> temp_q;
    pos_data_t temp;

    for (int i=0; i < 20; i++)
    {
        read_scan();
        if (i >= 10)    // discard the first 10 scan just incase
        {
            temp_q.push_back(data);
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

void Hokuyo_lidar::get_ui_CMD(UI_CMD_t in)
{
    _cmd = in;

    if (_cmd.set_type)  set_alt_type(_cmd.alt_type);
}

void Hokuyo_lidar::set_alt_type(lidar_alt_type dir)
{
    flag.alt = dir;

    if (dir == ROOF)
    {
        _tunnel_height = 0; // reset tunnel height
        calc_alt();

        _tunnel_height = abs(data.alt);
    }

    flag.printed_alt_mode = false;
    print_alt_type();

    _cmd.set_type = false;  // reset flag
}

void Hokuyo_lidar::set_max_scan_range(unsigned int range)
{
    // range input in m
    _max_scan_range = range*1000;
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
    ldata.pos.y = (cy/Area);//(cy/A);
    ldata.pos.z = (cz/Area);

    ldata.area = Area / (1000*1000);



} // end centroid
*/
