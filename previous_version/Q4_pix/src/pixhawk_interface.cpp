/*
 * pixhawk_interface.cpp
 *
 *  Created on: 20 Feb 2016
 *      Author: Calvin Vong
 */

#include "pixhawk_interface.h"
#include "wiringPi.h"

//general function


//---------------------------------------------------------------------
//				Pixhawk Interface Class
//---------------------------------------------------------------------
Pixhawk_Interface::
Pixhawk_Interface(){}

Pixhawk_Interface::
Pixhawk_Interface(Serial_Port *serial_port_)
{

	serial_port = serial_port_;
}

Pixhawk_Interface::
~Pixhawk_Interface()
{}




void Pixhawk_Interface::
read_msg()
{
    //printf("trying to read\n");
	bool success = false;
	//printf("come first\n");
	mavlink_message_t msg;
	while (!success)
    {
        success = serial_port->read_message(msg);
	}
	//printf("come here\n");
	if (success)
	{
        //printf("done\n");
		switch (msg.msgid)
		{
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				if (msg.sysid == 1)
				{
					static int N = 0;

					//send back an heartbeat
					mavlink_message_t msg_heartbeat;
					send_heartbeat(msg_heartbeat);

					//print out
					//printf("<3 beat %d, sys_ID: %d\n", count, msg.sysid);
					N++;
				}
				break;
			}

			case MAVLINK_MSG_ID_SYS_STATUS:
			{
				mavlink_sys_status_t sys_status;
				mavlink_msg_sys_status_decode(&msg, &sys_status);

				//print

				break;
			}

			case COMMAND_ACK:	//COMMAND_ACK #77
			{
				mavlink_command_ack_t ack;
				mavlink_msg_command_ack_decode(&msg, &ack);

				//print out
				//printf("Received command: %d, result: %d\n", ack.command, ack.result);

				break;
			}

			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			{
				//dont need to do anything
				break;
			}

			case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			{
				mavlink_rc_channels_raw_t rc_raw;
				mavlink_msg_rc_channels_raw_decode(&msg, &rc_raw);

                prev_ch5 = ch5;

                ch1 = rc_raw.chan1_raw;
                ch2 = rc_raw.chan2_raw;
                ch3 = rc_raw.chan3_raw;
                ch4 = rc_raw.chan4_raw;
                ch5 = rc_raw.chan5_raw;
                ch6 = rc_raw.chan6_raw;
                ch7 = rc_raw.chan7_raw;
                ch8 = rc_raw.chan8_raw;

				//printf("rc1 %d, rc2 %d, rc3 %d, rc4 %d, rc5 %d, rc6 %d, rc7 %d, rc8 %d\n", rc_raw.chan1_raw,rc_raw.chan2_raw,rc_raw.chan3_raw,rc_raw.chan4_raw, rc_raw.chan5_raw,rc_raw.chan6_raw,rc_raw.chan7_raw,rc_raw.chan8_raw);

				break;
			}
            case MAVLINK_MSG_ID_ATTITUDE:
            {
                mavlink_attitude_t alt;
                mavlink_msg_attitude_decode(&msg, &alt);

                pitch = alt.pitch*180/M_PI;
                roll = alt.roll*180/M_PI;
                yaw = alt.yaw*180/M_PI;

                //printf("pitch: %.2f, roll: %.2f, yaw: %.2f\n", pitch, roll, yaw);

                break;
            }
			case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:	//#36 from data stream RC_CHANNELS
			{

				break;
			}

			default:
			{
				/*
				 * 125: POWER_STATUS
				 * 253: STATUTEXT
				 */

				//print
				//printf("Other messages - msg ID: %d\n", msg.msgid);
				break;
			}
		}

	}

    return *pixdata;
}// end read_msg


// ----------------------------------------------------------------------------
//	Sending some heartbeat
// ----------------------------------------------------------------------------
void Pixhawk_Interface::
send_heartbeat(mavlink_message_t msg)
{
	mavlink_msg_heartbeat_pack(SYSTEM_ID, COMP_ID, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,0, 0, 0);

	serial_port->write_message(msg);

}

void Pixhawk_Interface::
get_home_pos()
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(SYSTEM_ID, COMP_ID, &msg,1,1,MAV_CMD_GET_HOME_POSITION,1,0,0,0,0,0,0,0);
    serial_port->write_message(msg);
}


void Pixhawk_Interface::
set_target_alt(int alt)
{
    mavlink_message_t msg;
    //------------------------------------------------------------------------  ---------- Frame 3 = gloable relative alt
    //mavlink_msg_command_long_pack(SYSTEM_ID, COMP_ID, &msg,1,1,MAV_CMD_DO_CHANGE_ALTITUDE,1,alt,3,0,0,0,0,0);
    mavlink_msg_command_long_pack(SYSTEM_ID, COMP_ID, &msg,1,1,MAV_CMD_CONDITION_CHANGE_ALT,1,2,0,0,0,0,0,alt);
    serial_port->write_message(msg);
}
// ----------------------------------------------------------------------------
//	Overwriting the RC channels
// ----------------------------------------------------------------------------
void Pixhawk_Interface::
write_RC_command(mavlink_message_t msg, int roll, int thr, int yaw)
{


	// MSG: #70 RC_CHANNELS_OVERRIDE
	//									                       target ID             pitch  throttle      ch5  6  7  8
	mavlink_msg_rc_channels_override_pack(SYSTEM_ID, COMP_ID, &msg,   1, 1,    roll,  0,       thr,    yaw,  0, 0, 0, 0);

	serial_port->write_message(msg);


	//debug print
	//printf("RC override sys ID %d, comp ID %d\n", SYSTEM_ID, COMP_ID);

}// end write RC command



// ----------------------------------------------------------------------------
//	Arming the quad
// ----------------------------------------------------------------------------
void Pixhawk_Interface::
arm_quad(mavlink_message_t msg, int arm)
{
	// MAV_CMD: #400 ARM/DISARM
	//													     target ID                                     ARM_CMD  unused param
	mavlink_msg_command_long_pack(SYSTEM_ID, COMP_ID, &msg,   1, 1,    MAV_CMD_COMPONENT_ARM_DISARM,   1,   arm,   1,1,1,1,1,1);


	serial_port->write_message(msg);

	//printf("ARM sys ID %d, comp ID %d\n", SYSTEM_ID, COMP_ID);

}// end arm quad



/*
void Pixhawk_Interface::
request_RC_raw(mavlink_message_t msg)
{
	mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMP_ID, &msg, 1, 1, MAV_DATA_STREAM_RC_CHANNELS, 100, 1);
	serial_port->write_message(msg);

}
*/

void Pixhawk_Interface::
request_pixhawk_info_msg()
{
    mavlink_message_t msg;
    //mavlink_msg_attitude_pack(SYSTEM_ID, COMP_ID, &msg, 0, 1,1,1,1,1,1);
    mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMP_ID, &msg, 1, 1,MAV_DATA_STREAM_ALL,100,1);
	serial_port->write_message(msg);
}



void Pixhawk_Interface::
start()
{
	//creating threads
	if ( pthread_create(&read_tid,NULL, &start_PI_read_thread, this) )
		printf("Error creating read thread!!\n");

	if ( pthread_create(&heartbeat_tid,NULL, &start_PI_heartbeat_thread, this) )
		printf("Error creating <3 beat thread!!\n");

/*
	if ( pthread_create(&read_rc_raw_tid,NULL, &start_PI_read_rc_raw_thread, this) )
		printf("Error creating read RC raw thread!!\n");
*/

	printf("Started Pixhawk interface threads\n");
}

void Pixhawk_Interface::
read_thread()
{
	int time = 0;

	while (1)
	{
       // printf("start read ");
        mavlink_message_t msg;
        request_pixhawk_info_msg();
		pix_data = read_msg(&pix_data);


		time = millis();
		//printf("hi\n");
		//delay(10);
		delayMicroseconds(500);
        //printf("read msg time: %d ms\n", millis()-time);
	}
}

void Pixhawk_Interface::
heartbeat_thread()
{
	while (1)
	{
 		long time = millis();

		mavlink_message_t msg;
		send_heartbeat(msg);

		delay(1000);
	//	printf("send <3 time: %ld ms\n", millis()-time);
	}
}

/*
void Pixhawk_Interface::
read_rc_raw_thread()
{
    while(1)
    {
        long time = millis();
        mavlink_message_t msg;
        request_RC_raw(msg);
        pix_data = read_msg(&pix_data);
        delay(10);
        //printf("read rc time %ld\n", millis() - time);
    }
}
*/

//------------------------------------------
//              THREAD
//------------------------------------------
/*
void* start_PI_read_rc_raw_thread(void* args)
{
    Pixhawk_Interface* pilot = (Pixhawk_Interface*) args;

    pilot->read_rc_raw_thread();

    return NULL;
}
*/

void* start_PI_read_thread(void* args)
{

	Pixhawk_Interface* pilot = (Pixhawk_Interface*) args;

	pilot->read_thread();

	return NULL;
}

void* start_PI_heartbeat_thread(void* args)
{
	Pixhawk_Interface* pilot = (Pixhawk_Interface*) args;

	pilot->heartbeat_thread();

	return NULL;
}

