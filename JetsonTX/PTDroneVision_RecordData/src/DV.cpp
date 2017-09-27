/***************************************/
/* DV Class */
/***************************************/

#include <DV.h>

// Namespaces
using namespace std;
using namespace sl;
using namespace cv;

bool runBool = false;
bool startBool = false;
bool resetBool = false;
bool exitBool = false;

//////////////////////////////////////////////////////////
/* CONSTRUCTOR & DESTRUCTOR */
//////////////////////////////////////////////////////////

// Constructor
DV::DV() : sl::Camera() {}

// Destructor
DV::~DV()
{
	std::cerr << "DV - Shutdown" << std::endl;
}

void DV::closeall()
{
	delete cameraPose;

	this->disableTracking();
	trackOpen = false;

	if(recordOpen) {
		while(recordState.status == false) sl::sleep_ms(1);

		this->disableRecording();
		recordOpen = false;
	}

	if(comOpen) {
		delete myComs;
		comOpen = false;
	}

	this->close();
}

//////////////////////////////////////////////////////////
/* INIT METHODS */
//////////////////////////////////////////////////////////

void DV::init()
{
	initVars();

	runParams.load("../Settings/DVCamSet");
	trackingParams.load("../Settings/DVCamSet");

	cameraPose = new sl::Pose();
}

void DV::initVars()
{
	frame = 1;

	imgRes = this->getResolution();

	imgWidth = imgRes.width;
	imgHeight = imgRes.height;

	camLeft2Centre = this->getCameraInformation().calibration_parameters.T.x *0.5f;

	camStartTimestamp = this->getCameraTimestamp();

	recordOpen = false;
	trackOpen = false;

	// File
	saveCamFile = false;
	saveExtFile = false;

	postfix = "_Set";

	camfileDetails = "Timestamp(ns);Rotation_X(rad);Rotation_Y(rad);Rotation_Z(rad);Position_X(mm);Position_Y(mm);Position_Z(mm);Confidence";

	fullpath = boost::filesystem::path(boost::filesystem::current_path());
	workDir = fullpath.string();

	resourcepath = boost::filesystem::path("/media/ubuntu/SD_card64/data/");//workDir + "/../Resources/");
	trackpath = boost::filesystem::path(resourcepath.string() + "Tracking/");
	recordpath = boost::filesystem::path(resourcepath.string() + "Records/");
	
	createDir(resourcepath);
}

//////////////////////////////////////////////////////////
/* START METHODS */
//////////////////////////////////////////////////////////

sl::ERROR_CODE DV::startTracking(const std::string &fname)
{
	createDir(trackpath);

	errTrack = this->enableTracking(trackingParams);

	if(errTrack != sl::SUCCESS) {
		std::cerr << "DV - Error starting tracking module: " << sl::errorCode2str(errTrack) << std::endl;
	}
	else {
		trackOpen = true;
		if(!createFile(outTrackname, trackpath, fname, ".csv", 0)) {
			saveCamFile = true;
		}
	}

	return errTrack;
}

//////////////////////////////////////////////////////////
/* SET METHODS */
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
/* GET METHODS */
//////////////////////////////////////////////////////////

int DV::getFrameNum()
{
	return frame;
}

sl::RuntimeParameters DV::getRunParams()
{
	return runParams;
}

//////////////////////////////////////////////////////////
/* FILE METHODS */
//////////////////////////////////////////////////////////

// Creating file
DV::ERR DV::createFile(std::string &outname, boost::filesystem::path dirpath, const std::string &fname, const std::string &fext, int details)
{
	static int num = 1;

	if (!fname.empty() && !fext.empty()) {

		std::ofstream outfile;

		outname = checkFilename(dirpath, fname+postfix, fext);

		outfile.open(outname);

		if (!outfile.is_open()) {
			std::cerr << "DV - Warning: Can't create CSV file. Launch the sample with Administrator rights" << std::endl;
		}
		else  {
			switch(details) {
			case(0) :	outfile << camfileDetails << std::endl; break;
			case(1)	:	outfile << lidarfileDetails << std::endl; break;
			case(2)	:	outfile << imufileDetails << std::endl; break;
			default	:	break;
			}
			outfile.close();

			std::cerr << "DV - Info: File Created: " << outname << std::endl;

			return SUCCESS;
		}
	}

	return FILE_NOT_FOUND;
}

// Save Camera data to file
DV::ERR DV::saveCamDataToFile(const std::string &fname, long long unsigned int camTimestamp, const sl::Vector3<float> &rotation, const sl::Vector3<float> &translation, int trackconfidence)
{	
	sprintf(text_save, "%llu;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%d", camTimestamp, rotation.x, rotation.y, rotation.z, translation.x, translation.y, translation.z, trackconfidence);

    printf("Depth displacement = %.2f mm, Lateral displacement = %.2f mm, Vertical displacement = %.2f mm", translation.z, translation.x, translation.y);
    
	if(saveCamFile && !fname.empty()) {
		std::ofstream outfile(fname, std::fstream::app);

		if(outfile.is_open()) {
		        outfile << text_save << "\n";
			outfile.close();

			return SUCCESS;
		}
	}

	return FILE_NOT_FOUND;
}

std::string DV::checkFilename(boost::filesystem::path dirpath, const std::string &fname, const std::string &fext)
{
	int i = 1;

	std::string filename = fname + std::to_string(i);

	std::ifstream infile(dirpath.string() + filename + fext);

	while(infile.is_open()) {
		infile.close();

		filename = fname + std::to_string(i);
		i++;
		infile.open(dirpath.string() + filename + fext);
	}

	return dirpath.string() + filename + fext;
}

//////////////////////////////////////////////////////////
/* FILE SYSTEM */
//////////////////////////////////////////////////////////

DV::ERR DV::createDir(boost::filesystem::path dirpath)
{
	if(!boost::filesystem::exists(dirpath)) {
		if(boost::filesystem::create_directory(dirpath)) {
			std::cerr << "DV - Info: Directory Created: " << dirpath.string() << std::endl;
		}
		else {
			std::cerr << "DV - Error Could not create directory at: " << dirpath.string() << std::endl;
			return NO_WRITE_PERMISSION;
		}
	}
	
	return SUCCESS;
}

boost::filesystem::path DV::createSubDir(boost::filesystem::path dirpath, const std::string &subDir)
{
	int i = 1;
	boost::filesystem::path dir;

	do {
		dir = boost::filesystem::path(dirpath.string() + subDir + std::to_string(i) + "/");
		i++;
	}
	while(boost::filesystem::exists(dir));

	if(createDir(dir)) return dir;
	else return dirpath;

}

//////////////////////////////////////////////////////////
/* METHODS */
//////////////////////////////////////////////////////////

void DV::transformPose(sl::Transform &pose, float tx) {

	sl::Transform transform_;

	transform_.setIdentity(); // Create the transformation matrix to separate camera frame and motion tracking frame

	transform_.tx = tx; // Move the tracking frame to the center of the camera

	pose = sl::Transform::inverse(transform_) * pose * transform_; // apply the transformation
}

//////////////////////////////////////////////////////////
/* EXTERN */
//////////////////////////////////////////////////////////

std::string DV::err2str(DV::ERR errcode)
{
	switch(errcode) {
		case(SUCCESS):	
				return "Success";
		case(FAILURE):	
				return "Failure";
		case(UNKNOWN):	
				return "Unknown error";
		case(FILE_NOT_FOUND):	
				return "File not found";
		case(NO_WRITE_PERMISSION):	
				return "No writing permission";
		case(NO_COM_PORT):	
				return "Invalid com port";
		default:	return "Undefined error code";
	}
}

//////////////////////////////////////////////////////////
/* MAIN GRABBER */
//////////////////////////////////////////////////////////

void DV::runGrab() 
{
	static int count = 0;

	if(!this->grab(runParams)) {

		// Get track info
		
		if(trackOpen) {
			trackState = this->getPosition(*cameraPose, sl::REFERENCE_FRAME_WORLD);
			camTimestamp = cameraPose->timestamp - camStartTimestamp;
			trackConfidence = cameraPose->pose_confidence;

			if(trackState == sl::TRACKING_STATE_OK) {

				transformPose(cameraPose->pose_data, camLeft2Centre);

				// Extract 3x1 rotation from pose
				rotation = cameraPose->getRotationVector();

				// Extract translation from pose
				translation = cameraPose->getTranslation();

			}
			
			if(trackConfidence == 0){
				count++;
				if(count > countTrackRst) {
					count = 0;

					std::cout << "Tracking Reset" << std::endl;

					this->resetTracking(cameraPose->pose_data);
				}
			}
			else count = 0;

			saveCamDataToFile(outTrackname, camTimestamp, rotation, translation, trackConfidence);
		
		}

		std::cout << frame << std::endl;
		frame++;

		
	}
	else sl::sleep_ms(1);

}

//////////////////////////////////////////////////////////
