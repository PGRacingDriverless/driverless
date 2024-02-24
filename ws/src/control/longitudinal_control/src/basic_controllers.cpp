#include "longitudinal_control/basic_controllers.h"

PID::PID(float Kp, float Ki, float Kd, float dT)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->dT = dT;
}

float PID::pTerm(float error)
{
    P = Kp * error;
    return P;
}

float PID::iTerm(float error)
{
    I = I + Ki * dT * error;
    return I;
}

float PID::dTerm(float error)
{
    D = (error - prev_error) /  dT;
    D = D * Kd;
    prev_error = error;
    return D;
}