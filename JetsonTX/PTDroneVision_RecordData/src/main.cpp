/***************************************/

// Includes
#include <iostream>
#include <thread>

#include <DV.h>

// Namespaces

using namespace std;
using namespace sl;
using namespace cv;

DV* DVCam;
sl::InitParameters initParams;

std::thread coms_call;

bool stopBool = false;

int counter = 0;

bool resetCode();
void runComs();

// Init

int main(int argc, char** argv)
{
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

	coms_call = std::thread(runComs);

	// Main loop
	while(1) {

		if(runBool) {
			DVCam->runGrab();
			//DVCam->printText("Hi my name is Jeff" + std::to_string(DVCam->getFrameNum()));
			
			/*if(DVCam->getFrameNum() > 100) {
				std::cout << "resetting" << std::endl;
				resetCode();
				std::cout << "reset done" << std::endl;
			}*/

		}
		else {
			
			if(!DVCam->grab(DVCam->getRunParams())) counter++;

			if(counter > countStart && startBool == true) {	

				sl::ERROR_CODE errTrack = DVCam->startTracking();

				if(errTrack != sl::SUCCESS) {
					std::cerr << "Shutting down!" << std::endl;
					DVCam->close();
					return 1;
				}

				//DVCam->flushlines();

				startBool = false;
				resetBool = false;
				runBool = true;
			}
		
			//DVCam->printText("READY");

			//if(counter == 200) startBool = true;		
		}

		if(exitBool) {
			exitBool = false;
			break;
		}
		else if(resetBool) if(!resetCode()) break;

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

	/*DV::ERR errCom = DVCam->startCommunication(port);

	if(errCom != DV::SUCCESS) {
		std::cerr << "DV - Error starting communication: " << DVCam->err2str(errCom) << std::endl;
		DVCam->close();
		return false;
	}

	DVCam->startRecordExternData();

	DVCam->flushlines();*/

	counter = 0;

	startBool = true;

	return true;	
}

void runComs()
{
  /*
	while(!stopBool) {
		//if(runBool) {
			
			DVCam->readData();
		//}
		sl::sleep_ms(1);
	}
  */

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
