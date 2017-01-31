#include "AP_controller.h"
#include "utility.h"

MedianFilter vel_z_filter(5,0);

//==============================================================//
//===================== Position CONTROLLER ====================//
//==============================================================//

// public functions
position_controller::position_controller()
{
    kp_pos_y = 1;
	ki_pos_y = 0;
	kd_pos_y = 0;
    _dt = 0.025;
}


void position_controller::update_controller_input(float y, float z, int CH_mode)
{

    _pos_y = y;
    _pos_z = z;


    // mode
    if (CH_mode > 1200) _flag_auto_mode = 1;
    else                _flag_auto_mode = 0;
}

int position_controller::update_y_pos_controller()
{
    float error = 0 - _pos_y;


    static float i_sum = 0;
    float i_term = 0;
    static float prev_error = 0;


    // integral
    if (!_flag_auto_mode)   i_sum = 0;  // reset integral when in manual mode

    i_sum += error * _dt;
    i_term = range_limiter(i_sum * ki_pos_y, MIN_I_ROLL, MAX_I_ROLL);

    // derivative
    float d_term = kd_pos_y * (prev_error - error) / _dt;
    prev_error = error;


    if (_flag_auto_mode)    return (int) 1500 + kp_pos_y * error + i_term + d_term;
    else                    return 0;
}

int position_controller::update_z_pos_controller()
{


    if (_flag_auto_mode)    return (int) 1500;
    else                    return 0;
}


// private
float position_controller::range_limiter(float in, float min_value, float max_value)
{
	if (in > max_value)		return max_value;
	else if (in < min_value)	return min_value;
	else                return in;
}

float position_controller::mode_switch_output_damping()
{


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
