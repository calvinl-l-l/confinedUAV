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

    // init variables
    _tunnel_height = 0;
    _max_scan_range = 6*1000;   // default 6m scan range

    _init_alt_type();   // TODO: test open space init for alt type
    //set_alt_type(FLOOR);    // TEMP

    data.range.reserve(1080);
    data.angle.reserve(1080);
    data.pc_y.reserve(1080);
    data.pc_z.reserve(1080);

    // init sin and cos look up table
    _init_tri_LUT();
}

void Hokuyo_lidar::read_scan()
{
    // Clear old data vector
    data.range.clear();

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

void Hokuyo_lidar::_update_pc()
{
    // clear old array
    data.pc_z.clear();
    data.pc_y.clear();
    data.angle.clear();

    int n = 0;
    double roll  = -_ph2_data.roll;     // in rad, roll angle from PH2
    double pitch = -_ph2_data.pitch;    // -ve due to lidar orientation

    // create tri func lookup
    double cos_roll  = cos(roll);
    double sin_roll  = sin(roll);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);

    for (int i=0; i < 540*2; i+=SCAN_DENSITY)
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

            // raw
            double temp_z = data.range[i] * _cosLUT[i] - _offset_z;
            double temp_y = data.range[i] * _sinLUT[i];

            // transformed
            double yt = cos_roll*temp_y - cos_pitch*sin_roll*temp_z + _offset_x*sin_roll*sin_pitch;
            double zt = sin_roll*temp_y + cos_pitch*cos_roll*temp_z - _offset_x*cos_roll*sin_pitch;

            data.pc_y.push_back((int) yt);
            data.pc_z.push_back((int) zt);
            n++;
        }
    }

    data.nyz = n;

    _data_loss = (float) n/1000*100;  // calc data loss, only 100
}


void Hokuyo_lidar::calc_alt()
{
    // angle sign use right hand rule
    double pitch = -_ph2_data.pitch;
    double roll  = _ph2_data.roll;     // -ve due to lidar orientation

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

        case TUNNEL:
            cout << "Stop using alt calculation\n";
            break;
    }


    data.alt = (unsigned int) temp/n;
    data.alt = (data.alt-_offset_z)*cos(pitch) - _offset_x*sin(pitch);

    if (flag.alt == ROOF)   data.alt = abs(_tunnel_height - data.alt);

    //printf("alt: %d\n", data.alt);   // debug
}


// to be removed
void Hokuyo_lidar::_get_centroid()
{
    double A = 0;
    double cy = 0;
    double cz = 0;

    for (int i=0; i <= data.nyz-1; i++)
    {
        A +=  (data.pc_y[i] * data.pc_z[i+1]) - (data.pc_y[i+1] * data.pc_z[i]);

        cy += (data.pc_y[i] + data.pc_y[i+1]) * (data.pc_y[i] * data.pc_z[i+1] - data.pc_y[i+1] * data.pc_z[i]);
        cz += (data.pc_z[i] + data.pc_z[i+1]) * (data.pc_y[i] * data.pc_z[i+1] - data.pc_y[i+1] * data.pc_z[i]);
    }

    A = fabs(0.5f * A);

    _ref_yc = cy/(6.0f * A);
    _ref_zc = cz/(6.0f * A);

    A /= 1000.0f*1000.0f;
    data.area = A;  // m^2

    cout << "Initial ground location is " << round(_ref_yc) << "mm of cetre horizontally\n";
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
            case 2:  a = "tunnel";
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

void Hokuyo_lidar::_init_tri_LUT()
{
    for (int i=0; i<1080; i++)
    {
        double rad = _urg.index2rad(i);

        _sinLUT[i] = sin(rad);
        _cosLUT[i] = cos(rad);
    }
}
