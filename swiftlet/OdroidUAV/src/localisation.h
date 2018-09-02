#ifndef _LOCALISATION_H_
#define _LOCALISATION_H_

// NOTE: Current implement only computes HT for 0 and 90 degree
// TODO: implement option for full HSM

#include <vector>
#include <math.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "lidar.h"
#include "../lib/findpeaks/findpeaks.h"

#define MAX_RHO 	5000 	// sufficient for container case
#define STEP_THETA 	0.5		// 0.5 degree step size
#define STEP_RHO	10		// 10mm step size
#define MAX_dRHO	500		// sufficient for container

using namespace std;
using namespace cv;

class localisation: public Hokuyo_lidar
{
public:

	localisation();
	void init();
	void run();	// Hough scan matching: return (dy, dz)

private:

	int _n_rho;

	vector<int> _Href;	// HT of reference scan

	vector<int>::const_iterator _bref;
	vector<int>::const_iterator _lref;

	pos_t _prev_pos;

	// HSM
	void _get_ref_scan();

	unsigned int _xcorr_cv(vector<int> sref, vector<int> ssrc, char yz);
	unsigned int _xcorr(vector<int> s1, vector<int> s2, int max_delay); // only returns index of max r
	vector<unsigned int> _xcorr_fast(vector<int> s1, vector<int> s2, int max_delay);

	vector<int> _DHT(vector<int> y, vector<int> z);

	vector<float> _mat_int2vector(Mat in);
	Mat _vector_int2mat(vector<int> in);

};


#endif
