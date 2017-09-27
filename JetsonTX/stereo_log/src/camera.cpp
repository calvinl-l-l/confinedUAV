#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include "jetsonGPIO.h"
#include <thread>
#include <fstream>

using namespace cv;
using namespace std;

unsigned int odroid_init = 0;
string userIN;

string generate_filaname(const string &path);

//==== THREAD ============================================================
void ui_thread()
{
  getline(cin, userIN);
  //userIN = "";
}

//==== MAIN ==============================================================
int main()
{
  // GPIO init
  gpioUnexport(gpio187);
  gpioExport(gpio187);
  gpioSetDirection(gpio187, inputPin);

  thread ui(ui_thread);
  VideoCapture cap(0);
  
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

  int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  if (!cap.isOpened())
  {
	cout << "shit" << endl;
	return -1;
  }

  //namedWindow("Selfie", 1); // for debugging

  int n = 0;

  string filename = generate_filaname("/media/ubuntu/SD_card64/video/");
  //string filename = generate_filaname("../video/");
  VideoWriter vid(filename, CV_FOURCC('M','J','P','G'), 30, Size(width, height), true);

  while (1)
  {
    static int prev_init = 0;
    Mat frame;

    cap >> frame;

    gpioGetValue(gpio187, &odroid_init);

    if ((!odroid_init && prev_init) || userIN == "stop")
    {
      cout << "Stopped recording . . ." << endl;

      prev_init = 0;
      vid.release();
      cout << "Please restart program . . ." << endl;
      break;
    }
    else if (odroid_init || userIN == "start")
    {
      if (!prev_init) cout << "Started recording . . ." << endl;

      vid.write(frame);
      prev_init = 1;
    }

      //imshow("Selfie", frame);  // for debugging
  }

  ui.join();
  return 0;
}

string generate_filaname(const string &path)
{
	int i = 1;

	std::string filename = "stereo_" + std::to_string(i);

	std::ifstream infile(path+ filename + ".avi");

	while(infile.is_open()) {
		infile.close();

		filename = "stereo_" + std::to_string(++i);
		
		infile.open(path + filename + ".avi");
	}

	return path + filename + ".avi";
}
