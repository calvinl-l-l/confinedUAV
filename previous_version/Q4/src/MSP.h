// Multiwii Serial Protocal
#pragma once

#include <errno.h>
#include <iostream>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <wiringSerial.h>
#include "utility.h"

// Multiwii Serial Protocal
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

// GENERAL VARIABLE
#define DATA_SIZE   64

/*
#define IDLE                0
#define	HEADER_M            1
#define	HEADER_ARROW        2
#define	HEADER_SIZE         3
#define	HEADER_CMD          4
#define	HEADER_DATA         5
#define	MESSAGE_RECEIVED    6
*/
using namespace std;

struct MSP_packet_t
{
    uint8_t data_length;
    uint8_t CMD;
    uint8_t inBuf[DATA_SIZE];
    uint8_t checksum;
    uint8_t idx;

};


static enum mspState_e{
	IDLE,
	HEADER_M,
	HEADER_ARROW,
	HEADER_SIZE,
	HEADER_CMD,
	HEADER_DATA,
	MESSAGE_RECEIVED
}c_state;


// MSP class
class MSP
{
public:

// variable
    int sp;

// data structure
    struct attitude_t
    {
        float roll;
        float pitch;
        float yaw;
    }attitude;

    struct RC_t
    {
        uint16_t ch1;   // roll
        uint16_t ch2;   // pitch
        uint16_t ch3;   // yaw
        uint16_t ch4;   // throttle
        uint16_t ch5;
        uint16_t ch6;
        uint16_t ch7;
        uint16_t ch8;
    }rcIn, rcOut;

    struct motor_t
    {
        uint16_t m1;
        uint16_t m2;
        uint16_t m3;
        uint16_t m4;
    }motor;

// function
	MSP(char* port, int baud);
	void send_msg(uint8_t CMD, uint16_t data[], uint8_t n_bytes, char mode);
    void read_msg();
    void decode_msg();

private:

	//mspState_e c_state;
	//int c_state;
	MSP_packet_t msg;

	void encode_data(uint16_t idata[], uint8_t *odata, int len);
    uint8_t mspSerialChecksum(uint8_t checksum, uint8_t byte);
	uint8_t mspSerialChecksumBuf(uint8_t checksum, uint8_t *data, int len);
};

// helper function
int16_t uI8_toINT16(uint8_t buf[], int idx);
uint16_t uI8_toUINT16(uint8_t buf[], int idx);


