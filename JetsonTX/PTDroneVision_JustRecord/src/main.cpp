/***************************************/

// Includes
#include <iostream>
#include <thread>
#include "jetsonGPIO.h"
#include "DV.h"

// Namespaces

using namespace std;
using namespace sl;
using namespace cv;

DV* DVCam;
sl::InitParameters initParams;

std::thread coms_call;

bool stopBool = false;

int counter = 0;
unsigned int odroid_init = 0;
bool resetCode();
void runComs();
void log_data();
// Init

int main(int argc, char** argv)
{
  // GPIO setup

  gpioUnexport(gpio187);
  gpioExport(gpio187);
  gpioSetDirection(gpio187, inputPin);

  runBool = false;

	if(sl::Camera::isZEDconnected() == 0) {
		std::cerr << "Cannot find zed camera!" << std::endl;
		return 1;
	}
	std::cout << "Hi" << std::endl;

	DVCam = new DV();

	initParams.load("../Settings/DVCamSet");

	sl::ERROR_CODE errInit = DVCam->open(initParams);

	if(errInit != sl::SUCCESS) {
		std::cerr << "DV - Error starting camera module:" << sl::errorCode2str(errInit) << std::endl;
		DVCam->close();
		return 1;
	} 

	DVCam->init();

	// Enable recording
	sl::ERROR_CODE errRecord = DVCam->startRecording(1);

	if(errRecord != sl::SUCCESS) {
		std::cerr << "Shutting down!" << std::endl;
		DVCam->close();
		return 1;
	}
  
	coms_call = std::thread(runComs);

	// Main loop
	while(1) 
	{

		if(runBool) {
			DVCam->runGrab();
		}
		else {
			
			if(!DVCam->grab(DVCam->getRunParams())) counter++;

			if(counter > countStart && startBool == true) {	

				startBool = false;
				resetBool = false;
				runBool = true;
			}
		
			//DVCam->printText("READY");	
		}

		if(exitBool) {
			exitBool = false;
			break;
		}
		else if(resetBool) if(!resetCode()) break;

		log_data();
	}

	std::cerr << "Done!" << std::endl;

	stopBool = true;
	coms_call.join();

	delete DVCam;

	return 0;
}

bool resetCode()
{
	runBool = false;

	DVCam->closeall();

	sl::ERROR_CODE errInit = DVCam->open(initParams);

	if(errInit != sl::SUCCESS) {
		std::cerr << "DV - Error starting camera module:" << sl::errorCode2str(errInit) << std::endl;
		DVCam->close();
		return false;
	} 

	DVCam->init();

	// Enable recording
	sl::ERROR_CODE errRecord = DVCam->startRecording(1);

	if(errRecord != sl::SUCCESS) {
		std::cerr << "Shutting down!" << std::endl;
		DVCam->close();
		return false;
	}

	counter = 0;

	startBool = true;

	return true;	
}

void log_data()
{
	static int init = 0;
		
	gpioGetValue(gpio187, &odroid_init);
	if (odroid_init)
	{
		startBool = true;
		init = 1;
	}
	else if ((!odroid_init) && (init))
	{
		exitBool = true;
		init = 0;
	}
}
	




void runComs()
{

  while (1)
    {
      char a;
      cin >> a;

	if (a=='q')
	{
	  exitBool = true;
	  break;
	}
	else if(a == 'r') {
		startBool = true;
	}
	
      
    }

}

//////////////////////////////////////////////////////////
