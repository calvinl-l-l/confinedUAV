#include "localisation.h"

localisation::localisation()
{
	_n_rho = round(MAX_RHO/STEP_RHO) * 2 + 1;	// +1 because including 0
}

void localisation::init(vector<int> y, vector<int> z)
{
	_Href = _DHT(y, z);
	_bref = _Href.begin();
	_lref = _Href.begin()+_Href.size();
}

void localisation::run(vector<int> y, vector<int> z)
{
	// Hough transform
	vector<int> Hsrc = _DHT(y, z);

	// breaking the 1D vector into the 0 and 90 degree column
	vector<int>::const_iterator bsrc = Hsrc.begin();
	vector<int>::const_iterator lsrc = Hsrc.begin() + Hsrc.size();

	vector<int> Href0(_bref, _bref+_n_rho);
	vector<int> Hsrc0(bsrc, bsrc+_n_rho);
	vector<int> Href90(_bref+_n_rho+1, _lref);
	vector<int> Hsrc90(bsrc+_n_rho+1, lsrc);


	// T estimation
	unsigned int xc0  = _xcorr(Href0, Hsrc0, MAX_dRHO);
	unsigned int xc90 = _xcorr(Href90, Hsrc90, MAX_dRHO);

	pos.y = (xc0 - MAX_dRHO) * STEP_RHO;	// dy
	pos.z = (xc90 - MAX_dRHO) * STEP_RHO;	// dz
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

unsigned int localisation::_xcorr(vector<int> s1, vector<int> s2, int max_delay)
{
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
