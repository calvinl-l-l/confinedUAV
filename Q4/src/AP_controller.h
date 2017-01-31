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
#define MAX_I_ROLL       70
#define MIN_I_ROLL       -70

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

    float alt_target;

	// FUNCTIONS
	position_controller();

	void update_controller_input(float y, float z, int CH_mode);
	int update_y_pos_controller();
	int update_z_pos_controller();
	int arm();


private:
	// VARIABLES
	int _estimated_hover_throttle;

	float _dt;	// 10ms loop

	float _pos_y;
	float _pos_z;


	// flags
	int _flag_set_target_alt;
	int _flag_auto_mode;
	int _flag_arm;
	char _flag_prev_mode;

	//PID PID_vel_z(kp_pos_z, 0, kd_pos_z, _dt, 0, 0, MAX_ASCEND_VEL, MAX_DESCEND_VEL);
	//LowPassFilterFloat _vel_error_LPF;   // low-pass-filter on z-axis velocity error

	// FUNCTIONS
    float mode_switch_output_damping();
	float range_limiter(float in, float min_value, float max_value);
};



// PID class --------------------------------------------------------
class PID
{
public:
    PID();
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
