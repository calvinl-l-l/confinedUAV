/**************************************/
/* Coms Class */
/***************************************/

#ifndef __Coms_INCLUDE__
#define __Coms_INCLUDE__

// Includes
#include <iostream>

#include <serial/serial.h>

#define rnum 6

// Coms Class
class Coms : public serial::Serial {

public:

	enum ERR {
		SUCCESS,
		FAILURE,
		END_NOT_FOUND,
		INVALID_PACKET_LENGTH
	};

	char sSeparator = '/';
	char eSeparator = '#';
	char dSeparator = ';';

	void sendText(const std::string &action, const std::string &text);
	
	ERR readStream(std::string &datastream, std::string &rx, int length, int index);
	void readBuffer(std::string &rx, int &length, int numbytes);

	std::string err2str(ERR errcode);

	// Constructor
	Coms(const std::string &port, int baudrate);

	// Destructor
	~Coms();
	
private:
	std::string concatSend(const std::string &action, const std::string &data);
};

#endif /*__Coms_INCLUDE__*/ 
