#include "AP_controller.h"
#include "utility.h"


//==============================================================//
//===================== Position CONTROLLER ====================//
//==============================================================//

// public functions
position_controller::position_controller()
{
  kp_pos_y = 0.21;
	ki_pos_y = 0;
	kd_pos_y = 0.22;
  kp_pos_y_nw = 0;

	kp_pos_z = 1;

  _dt = 0.025;

	i_term = 0;
  d_term = 0;
  _prev_d_term = 0;
  roll_trim = 0;
}


void position_controller::update_controller_input(double y, double z, float wallL, float wallR, int CH_mode, int ch8)
{

  _pos_y = y;
  _pos_z = z;

  _d_wallR = wallR;
  _d_wallL = wallL;

  _ch8 = ch8;
// save previous mode
	if (_flag_prev_mode != _flag_auto_mode)	_flag_prev_mode = _flag_auto_mode;

  //  mode switching
  // > up pos   = manual mode
  // > mid pos  = change lateral setpoint
  // > down pos = auto mode
  if (CH_mode > 1650)
  {
    _flag_auto_mode = 1;
    _flag_set_setpoint = 0;
  }
  else if ((CH_mode > 1200)&&(CH_mode <= 1650))
  {
    _flag_auto_mode = 0;

    _flag_set_setpoint = 0;
    // set to 1 for changing lateral setpoint
    // set to 0 for manual mode but logging
  }
  else if (CH_mode <= 1200)
  {
    _flag_auto_mode = 0;
    _flag_set_setpoint = 0;
  }
}

int position_controller::update_y_pos_controller()
{
  // changing position setpoint
  if (_flag_set_setpoint) pos_y_setpoint = _pos_y;

  double error = 0 - _pos_y;  // can replae 0 with pos_y_setpoint

  static double prev_error = 0;

  // integral
  i_term += (double) ki_pos_y * error * _dt;
  i_term = range_limiter(i_term, MIN_I_ROLL, MAX_I_ROLL);

  if (!_flag_auto_mode || flag_outside_scan_boundary)   i_term = 0;  // reset integral when in manual mode

  // derivative
  double derror = error;//pos_y_error_filter.in(error);

  _prev_d_term = d_term;

  d_term = (double) kd_pos_y * (derror - prev_error) / _dt;
  prev_error = derror;

  d_term = range_limiter(d_term, MIN_D_ROLL, MAX_D_ROLL);

  d_term = (1-D_smoothing_factor) * _prev_d_term + D_smoothing_factor * d_term;

  double d_out = d_term;
  if (fabs(d_term) < 15)	d_out = 0;

	int output = range_limiter((int) 1500 + kp_pos_y * error + i_term + d_out, MIN_ROLL_OUT, MAX_ROLL_OUT);
  output += roll_trim;
  //roll_PWMout = output;


	if (_flag_auto_mode && !flag_outside_scan_boundary)
  {
    roll_PWMout = manual_override(mode_switch_output_damping(output));
    return manual_override(mode_switch_output_damping(output));
  }
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
        else            return manual_override((int) range_limiter(output, 1100, 1900));
    }
    else                return 0;
}


// private
int position_controller::manual_override(int ctrl_output)
{
	if (_ch8 >= 1600 || _ch8 <= 1400)
	{
		return 0;
	}
	else
	{
		return ctrl_output;
	}

}

double position_controller::range_limiter(double in, double min_value, double max_value)
{
	if (in > max_value)		return max_value;
	else if (in < min_value)	return min_value;
	else                return in;
}

int position_controller::mode_switch_output_damping(int signal_out)
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
