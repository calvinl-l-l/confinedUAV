#include "control.h"


// controller .cpp
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
    integral += (float) error * dt;
    float derivator = (float) (error - perror)/dt;

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
