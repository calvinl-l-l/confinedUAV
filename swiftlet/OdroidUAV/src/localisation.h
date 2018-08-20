#ifndef _LOCALISATION_H_
#define _LOCALISATION_H_

// NOTE: Current implement only computes HT for 0 and 90 degree
// TODO: implement option for full HSM

#include <vector>
#include <math.h>
#include <iostream>

#define MAX_RHO 	5000 	// sufficient for container case
#define STEP_THETA 	0.5		// 0.5 degree step size
#define STEP_RHO	10		// 10mm step size
#define MAX_dRHO	500		// sufficient for container

using namespace std;

class localisation
{
public:
	struct pos_t
	{
		int y;	// mm
		int z;	// mm
	};

	pos_t pos;

	localisation();
	void init(vector<int> y, vector<int> z);
	void run(vector<int> y, vector<int> z);	// Hough scan matching: return (dy, dz)

private:
	int _n_rho;

	vector<int> _Href;	// HT of reference scan
	vector<int>::const_iterator _bref;
	vector<int>::const_iterator _lref;

	// HSM
	unsigned int _xcorr(vector<int> s1, vector<int> s2, int max_delay); // only returns index of max r
	vector<int> _DHT(vector<int> y, vector<int> z);

};


#endif
