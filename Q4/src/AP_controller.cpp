#include "AP_controller.h"
#include "utility.h"

MedianFilter vel_z_filter(5,0);

//==============================================================//
//===================== Position CONTROLLER ====================//
//==============================================================//

// public functions
position_controller::position_controller()
{
    kp_pos_y = 0.1;
	ki_pos_y = 0.03;
	kd_pos_y = 0;
    kp_pos_y_nw = 0;


	kp_pos_z = 1;

    _dt = 0.025;
}


void position_controller::update_controller_input(float y, float z, float wallL, float wallR, int CH_mode)
{

    _pos_y = y;
    _pos_z = z;

    _d_wallR = wallR;
    _d_wallL = wallL;


	// save previous mode
	if (_flag_prev_mode != _flag_auto_mode)	_flag_prev_mode = _flag_auto_mode;

    // mode
    if (CH_mode > 1200) _flag_auto_mode = 1;
    else                _flag_auto_mode = 0;
}

int position_controller::update_y_pos_controller()
{
    float error = 0 - _pos_y;

    int kp;
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


    // near wall condition
    if (_d_wallL <= 300 || _d_wallR <= 300) kp = kp_pos_y + kp_pos_y_nw;
    else                                    kp = kp_pos_y;


	int output = range_limiter((int) 1500 + kp * error + i_term + d_term, 1100, 1900);


	if (_flag_auto_mode)    return mode_switch_output_damping(output);
    else                    return 0;
}

int position_controller::update_z_pos_controller()
{
    float error = _pos_z;

    int output = 1500;

    if (error > 100)
    {
        output += error * kp_pos_z;
    }
    else if (error < -100)
    {
        output -= error * kp_pos_z;
    }
    else    {output = 0;}   // within deadzone then dont do anything


    if (_flag_auto_mode)
    {
        if (output==0)  return 0;
        else            return (int) range_limiter(output, 1100, 1900);
    }
    else                return 0;
}


// private
float position_controller::range_limiter(float in, float min_value, float max_value)
{
	if (in > max_value)		return max_value;
	else if (in < min_value)	return min_value;
	else                return in;
}

float position_controller::mode_switch_output_damping(int signal_out)
{
	static int hold_PID = 0;
	static char dir = 'R';
	static int output = 1500;

	if (!_flag_prev_mode && _flag_auto_mode && !hold_PID)
	{
		if (1500 > signal_out)	dir = 'R';
		else					dir = 'L';
	}

	if (hold_PID)
	{
		if (dir == 'R')
		{
			output += 10;

			if (output >= signal_out)	hold_PID = 0;

		}
		else if (dir == 'L')
		{
			output -= 10;

			if (output <= signal_out)	hold_PID = 0;
		}

		return output;
	}
	else
	{
		output = 1500;
		return signal_out;
	}

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
