#include "lidar.h"


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
       read(0);
       start_area += area;
   }

   start_area /= 10.0f;

}

void Hokuyo_lidar::set_startup_time()
{
    ts_startup = t_temp;

}

void Hokuyo_lidar::read(float roll)
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

    //long t_temp;
    if (!urg.get_distance(range, &t_temp))
    {
      _flag_lidar_error  = 1;
      cout << "Error reading lidar scan!" << endl;
    }
    else
    {
      _flag_lidar_error = 0;
    }

    ts = t_temp - ts_startup;    // NULL = no geting time stamp

    // removing outliers
    //vector<int> idx = lonely_pts_detector();
    vector<int> idx;  // for disabling lonely point detector, need to comment out the lonely function too

    // converting raw data to cartesian
    int i2 = 0; // index for removing idx vector
    int n = 0;
    for (int i=0; i < 540*2; i++)
    {
        // angle of the range
        double rad = urg.index2rad(i);
        angle.push_back(urg.index2deg(i));

        if (idx.size() > 0 && idx[i2] == i && i2 < idx.size())
        {
          i2++;
          continue;
        }
        else
        {
          // only store meaningfull data
          if ((fabs(angle[i]) > 81.0f && fabs(angle[i]) < 95.0f) || angle[i] > 125.0f)
          {
              // condition is for the QAV500 frame
              // redundant: can rely on lonely_pts_detector
          }
          else
          {
            if ((range[i] > 350) && (range[i] <= 10000))  // redundant
            {
            	z.push_back((double) (range[i] * cos(rad)));
              y.push_back((double) (range[i] * sin(rad)));
            	n++;
            }
          }
        }
    }


    nyz = n;

    _data_loss = (float) n/1080;  // calc data loss

   // cout << "nyz " << nyz << endl;
    // rotation (y,z)
    for (int i=0; i < nyz; i++)
    {
        double y_temp = cos(roll*M_PI/180.0f) * y[i] - sin(roll*M_PI/180.0f) * z[i];
        double z_temp = sin(roll*M_PI/180.0f) * y[i] + cos(roll*M_PI/180.0f) * z[i];

        y[i] = y_temp;
        z[i] = z_temp;
    }

    //get_symmetry_pt();
    // overriding get_symmetry_pt for QAV500
    yz_start_pt = 0;
    yz_end_pt = nyz;


    // *** hack ***
    get_dist2wall(roll);
    // *** hack ***

    //get_centroid1();
    get_centroid2();

    pos_loc_y3 = (dist_wallL - dist_wallR)/2.0f;
    //cout << "y2  " << setw(10) << pos_loc_y2 << " y3 " << setw(10) << pos_loc_y3 << " R " << setw(10) << dist_wallR << endl;
}  // end of lidar read();


// centroid
void Hokuyo_lidar::get_centroid1()
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



} // end centroid

void Hokuyo_lidar::get_centroid2()
{
    double A = 0;
    double cy = 0;
    double cz = 0;

    for (int i=yz_start_pt; i <= yz_end_pt; i++)
    {
        A +=  (y[i] * z[i+1]) - (y[i+1] * z[i]);

        cy += (y[i] + y[i+1]) * (y[i] * z[i+1] - y[i+1] * z[i]);
        cz += (z[i] + z[i+1]) * (y[i] * z[i+1] - y[i+1] * z[i]);
        //cout << cy << endl;
    }

    A /= 2.0f;


    pos_loc_y2 = cy/(6.0f * A);
    pos_loc_z2 = cz/(6.0f * A);

    //cout << pos_loc_y2 << endl;

    A /= -(1000.0f*1000.0f);
    area = A;
    //cout << pos_loc_y2 << ' ' <<  A << endl;
}

void Hokuyo_lidar::get_symmetry_pt()
{
    yz_start_pt = 0;
    yz_end_pt = nyz - 1;

    // check if points are connected
    // left side
    for (int i = 20; i > 0; i--)
    {
        // 86mm is threshold, max vertical distance between points, assuming 10m range
        if (fabs(z[i] - z[i-1]) <= 86)
        {
            yz_start_pt = i - 1;
        }
        else    {break;}
    }

    // right side
    for (int i = nyz-21; i < nyz - 1; i++)
    {
        // 86mm is threshold, max vertical distance between points, assuming 10m range
        if (fabs(z[i] - z[i+1]) <= 86)
        {
            yz_end_pt = i + 1;
        }
        else    {break;}
    }

    // compare which side is lower
    if (z[yz_start_pt] > z[yz_end_pt])
    {
        for (int i=nyz-1; i > (int) nyz/2; i--)
        {
            if ((z[i] >= z[yz_start_pt]) && (z[i-1] >= z[yz_start_pt]))
            {
                yz_end_pt = i;
                break;
            }
        }
    }
    else if (z[yz_start_pt] < z[yz_end_pt])
    {
        for (int i=0; i < (int) nyz/2; i++)
        {
            if ((z[i] >= z[yz_end_pt]) && (z[i+1] >= z[yz_end_pt]))
            {
                yz_start_pt = i;

  break;
            }
        }
    }
}

void Hokuyo_lidar::get_altitude(char alt_ref, float roll)
{
    // two altitude mode:
    // 1. alt relative to top
    // 2. Est. alt from ground

	static float dist = 0;
	int n = 0;

	for (int i=-5; i <= 5; i++)
	{
		float temp = range[urg.deg2index(i-roll)] * cos(M_PI*(i)/180);

		if (temp)
		{
			dist += temp;
			n++;
		}
	}

	alt.alt_ref = alt_ref;

	alt.dist = (float) dist/n;
	dist = 0;


}


void Hokuyo_lidar::get_dist2wall(float roll)
{
    int nl = 0;
    int nr = 0;
    float i = 95;
    float distR = 0;
    float distL = 0;


    while (i <= 105)
    {
        // distance to right wall
        float temp_R = range[urg.deg2index(-i+roll)] * cos(M_PI*(90-i)/180);

        if ((temp_R >= 350) && (temp_R <= 10000))
        {
            nr++;
            distR += temp_R;
        }

        // distance to left wall
        float temp_L = range[urg.deg2index(i+roll)] * cos(M_PI*(90-i)/180);

        if ((temp_L >= 350) && (temp_L <= 10000))
        {
            nl++;
            distL += temp_L;
        }

        i += 0.25;
    }

    i = 81;
    while (i > 71)
    {
        // distance to right wall
        float temp_R = range[urg.deg2index(-i+roll)] * cos(M_PI*(90-i)/180);

        if ((temp_R >= 350) && (temp_R <= 10000))
        {
            nr++;
            distR += temp_R;
        }

        // distance to left wall
        float temp_L = range[urg.deg2index(i+roll)] * cos(M_PI*(90-i)/180);

        if ((temp_L >= 350) && (temp_L <= 10000))
        {
            nl++;
            distL += temp_L;
        }

        i -= 0.25;
    }

    dist_wallR = distR/nr;
    dist_wallL = distL/nl;

    //cout << "L: " << dist_wallL << " R: " << dist_wallR << " totall " << dist_wallL + dist_wallR << endl;
}

int Hokuyo_lidar::lidar_check_outof_boundary()
{
/*
    TO DO:
        extra condition on the huge change in centroid
*/

    // trigger flag (flag=1) when out of boundary

    float dA = fabs(start_area - area) / start_area*100;

    //if (dA > 50)    return 0;

    if ( (_data_loss <= 0.38f) || (area > 15) || _flag_lidar_error )
    {
      return 1;
    }
    else
    {
      return 0;
    }
    //cout << _data_loss << "  a" << area << endl;
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
    if (range[i] > rmax || range[i] < rmin) I.push_back(i);
  }

  return I;
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
