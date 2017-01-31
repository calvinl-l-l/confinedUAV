#include "AP_controller.h"
#include "utility.h"

MedianFilter vel_z_filter(5,0);

//==============================================================//
//===================== Position CONTROLLER ====================//
//==============================================================//

// public functions
position_controller::position_controller()
{
	kp_pos_z = 1.5;
	ki_pos_z = 1;
	kd_pos_z = 0;
	kp_vel_z = 1;
	kd_vel_z = 0.1;
	_dt = 0.025;

	_flag_set_target_alt = 0;
	_flag_alt_mode = 0;

	_estimated_hover_throttle = 1350;
}

void position_controller::update_pos(float y, float z, float alt_ceil, float alt_floor)
{
	// all divided by 10, from input (mm) to cm
	_pos_y = y/10.0f;
	_pos_z = z;
	_alt_ceil = alt_ceil;
	_alt_floor = alt_floor;
}

void position_controller::get_target_alt(int THR_input, int alt_mode_switch)
{
	if ((alt_mode_switch < 1800) && !_flag_set_target_alt)	// ch7 control alt mode
	{
		_flag_alt_mode = 1;
		_flag_set_target_alt = 1;

		_pos_target_z = _pos_z;
	}

	if ((THR_input > 1600) || (THR_input < 1400))
	{
		_pos_target_z = _pos_z;
	}

	if (alt_mode_switch > 1800)
	{
		_flag_alt_mode = 0;
		_flag_set_target_alt = 0;
	}

}

void position_controller::update_z_controller(uint16_t CH_out[8], int THR_input)
{
	// pos error to desired velocity ---------------------------------------
	float pos_z_error = _pos_target_z - _pos_z; //to_discrete(_pos_target_z - _pos_z, 5);

    cout << "pos_err:" << pos_z_error <<' '<< "_pos_target_z:" << _pos_target_z <<' ' ;

	_vel_target_z = sqrt_controller(pos_z_error, kp_pos_y, MAX_CLIMB_ACC); // selective p controller
	_vel_desired_z = throttle_to_desired_vel_z(THR_input);

	//_vel_target_z += _vel_desired_z;

    //cout << "_vel_target_z: " << _vel_target_z << ' ';


	// limit desired z velocity range
	_vel_target_z = range_limiter(_vel_target_z, MAX_ASCEND_VEL, MAX_DESCEND_VEL);

    cout << "target_vel: " << _vel_target_z << ' ';

	// deisred velocity to throttle output ----------------------------------
	float current_vel_z = vel_z_filter.in((_pos_z - _pos_last_z)/_dt);

	float vel_error_z = _vel_target_z - current_vel_z;

	// add in i term later
    cout << "current_vel_z: " << current_vel_z << ' ';

	float output = _estimated_hover_throttle + kp_vel_z * vel_error_z + kd_vel_z * (_vel_error_last_z - vel_error_z) / _dt;

    cout << "_vel_target_z: " << _vel_target_z << ' '  <<  "output: " << output << ' ' ;

	// limit throttle output
	if (_flag_alt_mode)		CH_out[0] = range_limiter(output, 1100, 1750);

	// update "last_" variables
	_vel_error_last_z = vel_error_z;
	_pos_last_z = _pos_z;
}


void position_controller::simple_PID_altHold(uint16_t CH_out[8])
{
    const int i_max = 300;
    const int i_min = -300;
    float pos_error_z = 60 - _pos_z; // 1m

    static float pos_error_last_z = 0;
    static float pos_error_sum_z = 0;

    pos_error_sum_z += pos_error_z * _dt ;

    if (pos_error_sum_z > i_max)          pos_error_sum_z = i_max; // limit integral
    else if (pos_error_sum_z < i_min)    pos_error_sum_z = i_min;


    float output =_estimated_hover_throttle + kp_pos_z * pos_error_z + ki_pos_z * pos_error_sum_z + kd_pos_z * (pos_error_last_z - pos_error_z)/_dt;

    pos_error_last_z = pos_error_z;


    if (_flag_alt_mode)
    {
        CH_out[0] = range_limiter(output, 1100, 1750);
        cout << "ouptut: " << CH_out[0] << " i-term: " << pos_error_sum_z;
    }
    else
    {
        pos_error_sum_z = 0; // reset if landed
    }
}


// private functions
float position_controller::sqrt_controller(float error, float kp, float second_ord_lim)
{
	if (second_ord_lim <= 0.0f || kp == 0) {
		return error*kp;
	}

	float linear_dist = second_ord_lim / (kp * kp);

	if (error > linear_dist) {
		return sqrtf(2.0f*second_ord_lim*(error - (linear_dist / 2.0f)));
	}
	else if (error < -linear_dist) {
		return -sqrtf(2.0f*second_ord_lim*(-error - (linear_dist / 2.0f)));
	}
	else {
		return error*kp;
	}

}

float position_controller::throttle_to_desired_vel_z(int THR_input)
{
	if ((THR_input > 1600) && (THR_input < 1700))	// ascend stage 1
	{
		return -STAGE1_CLIMB_VEL;
	}
	else if (THR_input >= 1700)						// ascend stage 1
	{
		return -STAGE2_CLIMB_VEL;
	}
	else if ((THR_input > 1300) && (THR_input < 1400))	// descend stage 1
	{
		return STAGE1_CLIMB_VEL;
	}
	else if (THR_input <= 1300)							// descend stage 2
	{
		return STAGE2_CLIMB_VEL;
	}
	else	{return 0;}	// return 0 if no pilot input

}

float position_controller::range_limiter(float in, float min_value, float max_value)
{
	if (in > max_value)		return max_value;
	else if (in < min_value)	return min_value;
	else                return in;
}
//==============================================================//
//============== END OF Position CONTROLLER ====================//
//==============================================================//



//==============================================================//
//========================== PID CONTROLLER ====================//
//==============================================================//
PID::PID(float _kp, float _ki, float _kd, float _dt, float _setpoint, float _MAX_INTEGRAL, float _llimit, float _ulimit)
{
	kp = _kp;
	ki = _ki;
	kd = _kd;

	dt = _dt;
	setpoint = _setpoint;
	MAX_INTEGRAL = _MAX_INTEGRAL;

	MAX_OUT = _ulimit;
	MIN_OUT = _llimit;

	perror = 0;
	integral = 0;
}


float PID::update(float in)
{
	float error = in - setpoint;
	integral += (float)error * dt;
	float derivator = (float)(error - perror) / dt;

	if (integral > MAX_INTEGRAL)    integral = MAX_INTEGRAL;

	float output = kp * error + ki * integral + kd * derivator;

	// limit output
	if (output>MAX_OUT)
		output = MAX_OUT;
	else if (output<MIN_OUT)
		output = MIN_OUT;

	perror = error;

	return output;
}

void PID::setPoint(float set_point)
{
	setpoint = set_point;
}

void PID::setKp(float p)
{
	kp = p;
}

void PID::setKi(float i)
{
	ki = i;
}

void PID::setKd(float d)
{
	kd = d;
}
//==============================================================//
//================== END OF PID CONTROLLER  ====================//
//==============================================================//


//------- random helper ------
int to_discrete(float in, int band)
{
    if (fmod(in, band) > (float) band/2)
        return in + band - fmod(in, band);
    else
        return in - fmod(in, band);
}
