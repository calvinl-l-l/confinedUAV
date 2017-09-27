/**************************************/
/* Coms Class */
/***************************************/

#include <Coms.h>

// Namespaces
using namespace std;

//////////////////////////////////////////////////////////
/* CONSTRUCTOR & DESTRUCTOR */
//////////////////////////////////////////////////////////

// Constructor
Coms::Coms(const std::string &port, int baudrate) : serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(700))
{}

// Destructor
Coms::~Coms()
{
	this->flush();

	this->close();
}

std::string Coms::concatSend(const std::string &action, const std::string &data)
{
	std::string senddata = action;
	int packetlen = data.length()+action.length()+5; 

	if(packetlen/100 == 0) senddata += "0";
	
	if(packetlen/10 == 0) senddata += "0";
	senddata += std::to_string(packetlen);
	
	senddata += sSeparator + data + eSeparator;
	
	return senddata;
}

void Coms::sendText(const std::string &action, const std::string &text)
{
	std::string senddata = concatSend(action, text);

	this->write(senddata);
}

Coms::ERR Coms::readStream(std::string &datastream, std::string &rx, int length, int index)
{
	int count = 0;
	int indr = index;

	std::string packetsize;

	if(index == length) {

		readBuffer(rx, length, rnum-2); // Header - actionlength
		indr = 0;
	}

	char curChar = rx[indr];

	while(curChar != sSeparator) {
		
		if(!isdigit(curChar)) return INVALID_PACKET_LENGTH;
		else if(indr == length) {
			readBuffer(rx, length, rnum);
			indr = 0;
		}
		else {
			packetsize += curChar;
			indr++;
		}

		curChar = rx[indr];
	}

	if(packetsize.length() == 0) return INVALID_PACKET_LENGTH;

	int packetLength = std::stoi(packetsize);
	indr++;

	if(indr == length) {

		int bufferReadLength = packetLength - (int) packetsize.length() - 3;

		if(bufferReadLength <= 0) return FAILURE;
	
		readBuffer(rx, length, bufferReadLength); // Size - sizelength - actionlength - separator

		if(rx[length-1] != eSeparator) return END_NOT_FOUND;
		else {
			curChar = rx[count];

			while(curChar != eSeparator) {

				datastream += curChar;
				count++;
				curChar = rx[count];
			}
		}
		
	}
	else {
		count = indr;
		curChar = rx[count];

		while(curChar != eSeparator) {

			if(count == length) {

				if(packetLength - count <= 0) return FAILURE;

				readBuffer(rx, length, packetLength - count);
				count = 0;

				if(rx[length-1] != eSeparator) return END_NOT_FOUND;
			}
			else {

				datastream += curChar;
				count++;
			}

			curChar = rx[count];
		}

	}

	this->flushInput();

	return SUCCESS;
}

void Coms::readBuffer(std::string &rx, int &length, int numbytes)
{
	rx = this->read(numbytes);
	length = rx.length();
}

std::string Coms::err2str(Coms::ERR errcode)
{
	switch(errcode) {
		case(SUCCESS):	
				return "Success";
		case(END_NOT_FOUND):	
				return "End operator not found";
		case(INVALID_PACKET_LENGTH):	
				return "Invalid packet length";
		default:	return "Undefined error code";
	}
}

//////////////////////////////////////////////////////////
