#ifndef MOTOR_H

#include <Servo.h>

#define MOTOR_H
#define STOP 1000

class Motor {
    public:
        Motor();
        void init(short pin);    
        
        void spin(int throttle, bool armed = false);
        void enableMotor(void);
        void disableMotor(void);

    private:
        Servo motor;
        short pin;
};

#endif