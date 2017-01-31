
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
}; // end of PID class

