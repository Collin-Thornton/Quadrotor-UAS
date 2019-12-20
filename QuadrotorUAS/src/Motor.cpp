#include "Motor.h"
#include <Arduino.h>

Motor::Motor() {}

void Motor::init(short pin) {
    this->pin = pin;
    motor.attach(pin);
    motor.writeMicroseconds(STOP);
}

void Motor::spin(int throttle, bool armed) {
    throttle = min(throttle, 2000);
    throttle = max(throttle, 1000);

    if(armed)   motor.writeMicroseconds(throttle);
    else        motor.writeMicroseconds(STOP);
}

void Motor::disableMotor(void) {
    motor.detach();
}

void Motor::enableMotor(void) {
    motor.attach(pin);
}