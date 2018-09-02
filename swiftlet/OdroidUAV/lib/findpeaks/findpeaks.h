//***************************************************
// peak finder 
//
// by Nathanael Yoder shared in Matlab File Exchange
//
// translated to c++ by claydergc
// link: https://github.com/claydergc/find-peaks.git 
//***************************************************

#ifndef FINDPEAKS_H
#define FINDPEAKS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#define EPS 2.2204e-16

using namespace std;

void findPeaks(vector<float> x0, vector<int>& peakInds);

#endif
