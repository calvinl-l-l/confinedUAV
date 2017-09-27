/***************************************/
/* DV Class */
/***************************************/

#ifndef __DV_INCLUDE__
#define __DV_INCLUDE__

// Includes
#include <iostream>
#include <fstream>
#include <sstream>

#include <vector>

// Zed
#include <sl/Camera.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>

#include <boost/filesystem.hpp>

#define countStart 10
#define countTrackRst 15

extern bool runBool;
extern bool startBool;
extern bool resetBool;
extern bool exitBool;

// Drone Vision Class //
class DV : public sl::Camera {

/////////////
/* PUBLIC */
/////////////
public:

//////////////////////////////////////////////////////////
/* ENUMERATIONS */
//////////////////////////////////////////////////////////
	enum ERR {
		SUCCESS,
		FAILURE,
		NOT_AVAILABLE,
		UNKNOWN,
		FILE_NOT_FOUND,
		NO_WRITE_PERMISSION,
		NO_NEW_DATA,
		NO_COM_PORT
	};

//////////////////////////////////////////////////////////
/* METHODS */
//////////////////////////////////////////////////////////

	sl::ERROR_CODE startRecording(int recordStep = 6, const std::string &fname = "SVOfile", sl::SVO_COMPRESSION_MODE compressMode = sl::SVO_COMPRESSION_MODE_LOSSLESS);

	int getFrameNum();
	sl::RuntimeParameters getRunParams();

	//Grab
	void runGrab();
	void init();

	std::string err2str(ERR errcode);

	void closeall();

	DV();
	~DV();

/////////////
/* PRIVATE */
/////////////
private:	

//////////////////////////////////////////////////////////
/* DECLARATIONS */
//////////////////////////////////////////////////////////

	int frame;

	std::string workDir;

	boost::filesystem::path fullpath;
	boost::filesystem::path resourcepath;
	boost::filesystem::path recordpath;

	sl::ERROR_CODE errInit;
	sl::ERROR_CODE errGrab;
	sl::ERROR_CODE errRecord;

	bool saveCamFile;
	bool saveExtFile;

	std::string camfileDetails;
    
	std::string outTrackname;
	std::string outSVOname;
	
	std::string postfix;

	char text_save[256];
	char out_text[256];

	sl::Resolution imgRes;

	int imgWidth;
	int imgHeight;

	float camLeft2Centre;

	sl::TRACKING_STATE trackState;

	int trackConfidence;

	bool recordOpen;

	int recFrame;

	sl::Pose* cameraPose;
	
	sl::RuntimeParameters runParams;

	sl::RecordingState recordState;

	long long unsigned int camStartTimestamp;

	long long unsigned int camTimestamp;

	sl::Vector3<float> rotation;
	sl::Vector3<float> translation;

//////////////////////////////////////////////////////////
/* METHODS */
//////////////////////////////////////////////////////////

	void initVars();

	void transformPose(sl::Transform &pose, float tx);

	// File system
	ERR createFile(std::string &outname, boost::filesystem::path dirpath, const std::string &fname, const std::string &fext, int details);
	ERR saveCamDataToFile(const std::string &fname, long long unsigned int camTimestamp, const sl::Vector3<float> &rotation, const sl::Vector3<float> &translation, int trackconfidence);

	ERR createDir(boost::filesystem::path dirpath);

	boost::filesystem::path createSubDir(boost::filesystem::path dirpath, const std::string &subDir);

	std::string checkFilename(boost::filesystem::path dirpath, const std::string &fname, const std::string &fext);

};

#endif /*__DV_INCLUDE__*/ 
