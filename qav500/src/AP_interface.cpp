#include "AP_interface.h"

AP_interface::AP_interface(Serial_Port *serial_port_)
{
	serial_port = serial_port_;

}


// ----------------------------------------------------------------------------
//	Sending some heartbeat
// ----------------------------------------------------------------------------
void AP_interface::send_heartbeat()
{
	mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SYSTEM_ID, COMP_ID, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,0, 0, 0);
	serial_port->write_message(msg);
}   // end send heartbeat


// ----------------------------------------------------------------------------
//	Overwriting the RC channels
// ----------------------------------------------------------------------------
void AP_interface::write_RC_command(int roll, int thr, int yaw)
{
	mavlink_message_t msg;

	// MSG: #70 RC_CHANNELS_OVERRIDE
	//									                       target ID             pitch  throttle      ch5  6  7  8
	//mavlink_msg_rc_channels_override_pack(SYSTEM_ID, COMP_ID, &msg,   1, 1,    roll,  0,       thr,    yaw,  0, 0, 0, 0);
//									                       target ID             THR   roll  pitch   yaw
	mavlink_msg_rc_channels_override_pack(SYSTEM_ID, COMP_ID, &msg,   1, 1,      roll,  0,   0,    yaw,  0, 0, 0, 0);
	serial_port->write_message(msg);


	//debug print
	//printf("RC override sys ID %d, comp ID %d\n", SYSTEM_ID, COMP_ID);

}// end write RC command


mavlink_data_t AP_interface::get_mavlink_data()
{
    while (mavlink_data_q.pop(mavlink_data)) {}

    return mavlink_data;
}


// ----------------------------------------------------------------------------
//	Read mavlink messages
// ----------------------------------------------------------------------------
void AP_interface::read_msg()
{
	bool success = false;

	mavlink_message_t msg;

  success = serial_port->read_message(msg);

	if (success)
	{

		switch (msg.msgid)
		{
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				if (msg.sysid == 1)
				{
					static int N = 0;

					//send back an heartbeat
					mavlink_message_t msg_heartbeat;
					//send_heartbeat();

					//print out
                    //printf("<3 beat %d, sys_ID: %d\n", N, msg.sysid);
					N++;
				}
				break;
			}
            case MAVLINK_MSG_ID_SYSTEM_TIME:
            {
                mavlink_system_time_t sys_time;
                mavlink_msg_system_time_decode(&msg, &sys_time);

                ts = sys_time.time_boot_ms;

            }

			case MAVLINK_MSG_ID_SYS_STATUS:
			{
				mavlink_sys_status_t sys_status;
				mavlink_msg_sys_status_decode(&msg, &sys_status);

				mavlink_data.batt_volt = sys_status.voltage_battery/1000.0f;
				mavlink_data.batt_current = sys_status.current_battery;
				mavlink_data.batt_remain = sys_status.battery_remaining;
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
			case MAVLINK_MSG_ID_RAW_IMU:
			{
				mavlink_raw_imu_t rimu;

				mavlink_msg_raw_imu_decode(&msg, &rimu);

				mavlink_data.imu.xacc_r = rimu.xacc;
				mavlink_data.imu.yacc_r = rimu.yacc;
				mavlink_data.imu.zacc_r = rimu.zacc;
				mavlink_data.imu.xgyro_r = rimu.xgyro;
				mavlink_data.imu.ygyro_r = rimu.ygyro;
				mavlink_data.imu.zgyro_r = rimu.zgyro;
				mavlink_data.imu.xmag_r = rimu.xmag;
				mavlink_data.imu.ymag_r = rimu.ymag;
				mavlink_data.imu.zmag_r = rimu.zmag;

				mavlink_data.imu.ts_imu_raw = rimu.time_usec;

				break;
			}
			case MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE:
			{

				break;
			}
			case MAVLINK_MSG_ID_HIGHRES_IMU:
			{
				mavlink_highres_imu_t himu;

				mavlink_msg_highres_imu_decode(&msg, &himu);

				mavlink_data.imu.xacc_h = himu.xacc;
				mavlink_data.imu.yacc_h = himu.yacc;
				mavlink_data.imu.zacc_h = himu.zacc;
				mavlink_data.imu.xgyro_h = himu.xgyro;
				mavlink_data.imu.ygyro_h = himu.ygyro;
				mavlink_data.imu.zgyro_h = himu.zgyro;
				mavlink_data.imu.xmag_h = himu.xmag;
				mavlink_data.imu.ymag_h = himu.ymag;
				mavlink_data.imu.zmag_h = himu.zmag;

				mavlink_data.imu.ts_imu_highres = himu.time_usec;

				break;
			}
			case MAVLINK_MSG_ID_RC_CHANNELS:
			{
				mavlink_rc_channels_t rc;
				mavlink_msg_rc_channels_decode(&msg, &rc);

				mavlink_data.ch9 = rc.chan9_raw;
				mavlink_data.ch1_PPM = rc.chan1_raw;

				break;
			}
			case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			{
				mavlink_rc_channels_raw_t rc_raw;
				mavlink_msg_rc_channels_raw_decode(&msg, &rc_raw);

        mavlink_data.prev_ch5 = mavlink_data.ch5;

        mavlink_data.ch1 = rc_raw.chan1_raw;
        mavlink_data.ch2 = rc_raw.chan2_raw;
        mavlink_data.ch3 = rc_raw.chan3_raw;
        mavlink_data.ch4 = rc_raw.chan4_raw;
        mavlink_data.ch5 = rc_raw.chan5_raw;
        mavlink_data.ch6 = rc_raw.chan6_raw;
        mavlink_data.ch7 = rc_raw.chan7_raw;
        mavlink_data.ch8 = rc_raw.chan8_raw;

        mavlink_data.ts_CH = rc_raw.time_boot_ms - mavlink_data.ts_startup;
				//printf("rc1 %d, rc2 %d, rc3 %d, rc4 %d, rc5 %d, rc6 %d, rc7 %d, rc8 %d, ts %ld\n",
        //        rc_raw.chan1_raw,rc_raw.chan2_raw,rc_raw.chan3_raw,rc_raw.chan4_raw,
        //        rc_raw.chan5_raw,rc_raw.chan6_raw,rc_raw.chan7_raw,rc_raw.chan8_raw,
        //        mavlink_data.ts_CH);

				break;
			}
      case MAVLINK_MSG_ID_ATTITUDE:
      {
          static unsigned long t0 = 0;

          mavlink_attitude_t alt;
          mavlink_msg_attitude_decode(&msg, &alt);

          mavlink_data.pitch = alt.pitch*180/M_PI;
          mavlink_data.roll = alt.roll*180/M_PI;
          mavlink_data.yaw = alt.yaw*180/M_PI;

          mavlink_data.ts_attitude = alt.time_boot_ms - mavlink_data.ts_startup;
          //cout << "timestamp: " << ts_attitude << endl;
          //printf("pitch: %.2f, roll: %.2f, yaw: %.2f\n", pitch, roll, yaw);
          //cout << "delay: " << micros() - t0 << endl;
      //    t0 = micros();

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

    mavlink_data_q.push(mavlink_data);

}   // end read mavlink messages

void AP_interface::close_sp()
{
    serial_port->close_serial();
}


void AP_interface::arm(int is_arm)
{
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(SYSTEM_ID, COMP_ID, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 1, is_arm, 1,1,1,1,1,1);

    serial_port->write_message(msg);
}

void AP_interface::reboot()
{
	mavlink_message_t msg;

	mavlink_msg_command_long_pack(SYSTEM_ID, COMP_ID, &msg, 1,1, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1,1,0,0,0,0,0,0);
	//mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t *msg, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
	serial_port->write_message(msg);
}

void AP_interface::set_startup_time()
{
    mavlink_data.ts_startup = ts;
}

void AP_interface::request_pixhawk_info_msg()
{
  mavlink_message_t msg;
  //mavlink_msg_attitude_pack(SYSTEM_ID, COMP_ID, &msg, 0, 1,1,1,1,1,1);
	mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMP_ID, &msg, 1, 1,MAV_DATA_STREAM_RC_CHANNELS,200,1);
	serial_port->write_message(msg);

	mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMP_ID, &msg, 1, 1,MAV_DATA_STREAM_RAW_SENSORS,200,1);
	serial_port->write_message(msg);
}