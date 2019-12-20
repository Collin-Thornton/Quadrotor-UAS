#ifndef PID_H
#define PID_H

class PID {
    public:
        PID(float kP, float kI, float kD, float minVal, float maxVal);
        float compute(float measured, float setpoint);

        float kP, kI, kD;

    private:
        float dt;
        long pastTime;
        float pastE, sumE, minVal, maxVal;
};
#endif