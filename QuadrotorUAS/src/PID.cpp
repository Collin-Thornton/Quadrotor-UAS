#include "PID.h"
#include <Arduino.h>

PID::PID(float kP, float kI, float kD, float minVal, float maxVal) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->minVal = minVal;
    this->maxVal = maxVal;
}

float PID::compute(float measured, float setpoint) {
    this->dt = (float)(micros() - pastTime) / 1000000.0;

    float error = setpoint - measured;
    this->sumE += error*this->dt;
    float eDot = (float)(error - pastE) / this->dt;

    float output = kP*error + kI*sumE + kD*eDot;
    output = min(output, this->maxVal);
    output = max(output, this->minVal);

    pastTime = micros();
    pastE = error;

    return output;
}
