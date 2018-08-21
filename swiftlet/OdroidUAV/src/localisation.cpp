#include "localisation.h"

localisation::localisation()
{
	lidar_init();	// initialise the lidar

	_n_rho = round(MAX_RHO/STEP_RHO) * 2 + 1;	// +1 because including 0
}

void localisation::init()
{
	_Href.clear();

	// TODO: get reference scan first

	_Href = _DHT(_data_ref.pc_y, _data.ref.pc_z);

}

void localisation::run()
{
	// Hough transform
	vector<int> Hsrc = _DHT(data.pc_y, data.pc_z);


	// T estimation
	vector<unsigned int> xc  = _xcorr(Href, Hsrc, MAX_dRHO);

	pos.y = (xc[0] - MAX_dRHO) * STEP_RHO;	// dy
	pos.z = (xc[1] - MAX_dRHO) * STEP_RHO;	// dz

	calc_alt();

	// pushing position data to the queue
	if (data_q.size() >= MAX_LDATA_QUEUE_SIZE) data_q.pop_front();    // limit memory usage
	data_q.push_back(ldata);
}

// discrete hough transform
vector<int> localisation::_DHT(vector<int> y, vector<int> z)
{
	int rho;

	vector<int> HT(_n_rho*2, 0);

	// 1D vector for 0 degree column followed by 90 degree column

	for (int t=0;t <= 90; t+=90)	// only consdier 0 and 90 degree
	{
		for (int i=0; i<y.size(); i++)
		{
			// 1D vector: 0->_n_rho-1 (0 degree column), _n_rho->2*_n_rho-1 (90 degree column)
			if (t==0)
			{
				rho = y[i];	// y*cos(t) + z*sin(t)

				// histogram
				int idx =  round((rho + MAX_RHO)/STEP_RHO);
				HT[idx]++;
			}
			else if (t==90)
			{
				rho = z[i]; // y*cos(t) + z*sin(t)

				// histogram
				int idx =  round((rho + MAX_RHO)/STEP_RHO);
				HT[idx + _n_rho]++;	// offset for 2nd column
			}
		}
	}
	cout << "size " << HT.size() << '\n';

	return HT;
}


vector<unsigned int> localisation::_xcorr_fast(vector<int> s1, vector<int> s2, int max_delay)
{
	// faster version of xcorr:
	// take 1D vecotor of two degree columns in HT, compute the xcorr in parallet in the same loop

	// only return max index of r
	// uncomment the r related line to get the full xcorr
	vector<unsigned int> result_idx;

	int r_max1 = 0;	// init foo max
	int r_max2 = 0;
	unsigned int max_idx0  = 0;
	unsigned int max_idx90 = 0;
	int idx = 0;	// index counter
	int n = s1.size();

	//vector<int> r;
	//r.reserve(n);	// init size of vector

	for (int d=-max_delay; d<=max_delay; d++)
	{
		int sum1 = 0; // temp sum
		int sum2 = 0;

		for (int i=0; i<n; i++)
		{
			int j = i + d;

			while (j<0)
				j += n;

			j %= n;

			sum1 += s1[i] * s2[j];
			sum2 += s1[i+_n_rho] * s2[j+_n_rho];
		}
		//r.push_back(s);

		if (sum1 >= r_max1)
		{
			r_max1 = sum1;
			max_idx0 = idx;
		}

		if (sum2 >= r_max2)
		{
			r_max2 = sum2;
			max_idx90 = idx;
		}

		idx++;
	}

	//r.push_back(max_idx);	// store the index of max r
	result_idx.push_back(max_idx0);
	result_idx.push_back(max_idx90);

	return result_idx;
}

unsigned int localisation::_xcorr(vector<int> s1, vector<int> s2, int max_delay)
{
	// generic xcorr

	// only return max index of r
	// uncomment the r related line to get the full xcorr

	vector<int> r;
	int r_max = 0;	// init foo max
	unsigned int max_idx = 0;
	int idx = 0;	// index counter
	int n = s1.size();
	//r.reserve(n);	// init size of vector

	for (int d=-max_delay; d<=max_delay; d++)
	{
		int s = 0; // temp sum

		for (int i=0; i<n; i++)
		{
			int j = i + d;

			while (j<0)
				j += n;

			j %= n;

			s += s1[i] * s2[j];

		}
		//r.push_back(s);

		if (s >= r_max)
		{
			r_max = s;
			max_idx = idx;
		}

		idx++;
	}

	//r.push_back(max_idx);	// store the index of max r

	return max_idx;
}
