#ifndef AP_CONTROLLER_H
#define AP_CONTROLLER_H

/********* NOTE **************
 	Using NED coordinate frame
	+z is down

	usage:
	update_pos();
	update_z_controller

	use z_controller_init() to reset everything (when landed)

******************************/
#include <math.h>
#include <string>
#include <iostream>

#include "utility.h"
#include "MedianFilter.h"
//#include "LowPassFilter.h"

#define MAX_ASCEND_VEL	-150// cm/s
#define	MAX_DESCEND_VEL	 150	// cm/s
#define STAGE1_CLIMB_VEL 50		// cm/s
#define STAGE2_CLIMB_VEL 100    // cm/s
#define MAX_CLIMB_ACC	 80	// cm/s^2
#define I_ACC_LIMIT	     50		// ?? random value
#define MAX_THROTTLE	 1800
#define MIN_THROTTLE	 1100	// air mode throttle

#define MAX_ROLL_OUT     1800
#define MIN_ROLL_OUT	 1200
#define I_ROLL_LIMIT     50

#define GROUND_ALT       40		// mm

// position controller class -------------------------------------------------------


using namespace std;

class position_controller
{
public:
	// VARIABLES
	float kp_pos_y;
	float ki_pos_y;
	float kd_pos_y;

	float kp_pos_z;
	float ki_pos_z;
	float kd_pos_z;
	float kp_vel_z;
	float kd_vel_z;

	// FUNCTIONS
	position_controller();
	void get_hover_throttle(int THR_input, float z);
	void update_pos(float y, float z, float alt_ceil, float alt_floor);
	void get_target_alt(int THR_input, int alt_mode_switch);
	void update_z_controller(uint16_t CH_out[8], int THR_input);
    void simple_PID_altHold(uint16_t CH_out[8]);
	//void updata_PID_gains(float kp, float ki, float kd, string var_to_upate);

private:
	// VARIABLES
	int _estimated_hover_throttle;

	float _dt;	// 10ms loop

	float _pos_y;

	float _pos_z;
	float _alt_ceil;
	float _alt_floor;
	float _pos_target_z;
	float _pos_last_z;

	float _vel_target_z;
	float _vel_desired_z;
	float _vel_error_last_z;



	// flags
	int _flag_set_target_alt;
	int _flag_alt_mode;

	//PID PID_vel_z(kp_pos_z, 0, kd_pos_z, _dt, 0, 0, MAX_ASCEND_VEL, MAX_DESCEND_VEL);
	//LowPassFilterFloat _vel_error_LPF;   // low-pass-filter on z-axis velocity error

	// FUNCTIONS
	float sqrt_controller(float error, float kp, float second_ord_lim);
	float throttle_to_desired_vel_z(int THR_input);
	float range_limiter(float in, float min_value, float max_value);
};



// PID class --------------------------------------------------------
class PID
{
public:
	PID(float _kp, float _ki, float _kd, float _dt, float _setpoint, float _MAX_INTEGRAL, float _llimit, float _ulimit);
	float update(float in);
	void setPoint(float set_point);
	void setKp(float p);
	void setKi(float i);
	void setKd(float d);

private:
	float setpoint;
	float kp;
	float ki;
	float kd;
	float dt;
	float MAX_INTEGRAL;
	float integral;
	float perror;
	float MAX_OUT;
	float MIN_OUT;
};

// random helper
int to_discrete(float in, int band);

#endif // ! AP_CONTROLLER_H
