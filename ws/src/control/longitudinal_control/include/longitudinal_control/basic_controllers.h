#pragma once

class PID
{
    private:
        float P = 0;
        float I = 0;
        float D = 0;
        float prev_error = 0;
        float pTerm(float error);
        float iTerm(float error);
        float dTerm(float error);
    public:
        float Kp, Ki, Kd;
        float dT;
        PID(float Kp, float Ki, float Kd, float dT);
        float get_control(float error)
        {
            P = pTerm(error);
            I = iTerm(error);
            D = dTerm(error);
            return P + I + D;
        }
};