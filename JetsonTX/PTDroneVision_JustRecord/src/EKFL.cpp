/**************************************/
/* DV Class */
/***************************************/

#include <EKLF.h>

// Namespaces
using namespace std;
using namespace cv;

//////////////////////////////////////////////////////////
/* CONSTRUCTOR & DESTRUCTOR */
//////////////////////////////////////////////////////////

// Constructor
EKLF::EKLF(const std::vector<float> &x_, const cv::Mat &P_, const cv::Mat &Q_, const cv::Mat &R_) 
{
	// sigma = 5 cm
	//sigma2 = 25; LIDAR
	x = x_;

	xMat = cv::Mat(x.size(), 1, CV_32FC1);
	memcpy(xMat.data, x.data(), x.size()*sizeof(float));

	P = P_;
	Q = Q_;
	R = R_;
}

// Destructor
EKLF::~EKLF()
{
	P.release();
	Q.release();
	R.release();
	G.release();
	FJ.release();
	HJ.release();
	Pp.release();
	Fx.release();
	Hx.release();
	xMat.release();
	zMat.release();

}

void EKLF::update(const std::vector<float> &z, const cv::Mat &FJ, const cv::Mat &HJ)
{
	zMat = cv::Mat(z.size(), 1, CV_32FC1);
	memcpy(zMat.data, z.data(), z.size()*sizeof(float));

	calcPriorPredict(FJ, P, Q);
	calcKalmanGain(Pp, HJ, R);
	calcEstimatedState(FJ, G, zMat, Hx);
	calcPosteriorPredict(G, HJ, Pp);

}
void EKLF::calcPriorPredict(const cv::Mat &Fk_1, const cv::Mat &Pk_1, const cv::Mat &Qk_1)
{
	Pp = Fk_1*Pk_1*Fk_1.t() + Qk_1;
}

void EKLF::calcKalmanGain(const cv::Mat &Pp, const cv::Mat &HJ, const cv::Mat &R)
{
	G = Pp*HJ.t()*(HJ*Pp*HJ.t() + R).inv();
}

void EKLF::calcEstimatedState(const cv::Mat &FJ, const cv::Mat &G, const cv::Mat &zMat, const cv::Mat &Hx)
{

	xMat = FJ + G*(zMat - Hx);

	
	if(xMat.isContinuous()) {
		x.assign((float*)xMat.datastart, (float*)xMat.dataend);
	}
	else {
		for (int i = 0; i < xMat.rows; ++i) {
		x.insert(x.end(), (float*)xMat.ptr<uchar>(i), (float*)xMat.ptr<uchar>(i)+xMat.cols);
		}
	}
}

void EKLF::calcPosteriorPredict(const cv::Mat &G, const cv::Mat &HJ, const cv::Mat &Pp)
{
	P = (cv::Mat::eye(G.rows, G.rows, CV_32FC1) - G*HJ)*Pp;
}

//////////////////////////////////////////////////////////
