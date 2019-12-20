#include <Arduino.h>

//#include "SerialInput.h"
#include "PID.h"
#include "Motor.h"
#include "IMU.h"
#include "Remote.h"
#include "Calibration.h"

Motor frontRight, backRight, backLeft, frontLeft;

Remote * Remote::instance[6];
Remote Throttle(7), Roll(2), Pitch(4), Yaw(8), SwitchOne(14), SwitchTwo(13);
State state; 

IMU * IMU::instance;
IMU imu;
Meas meas;
 
long pastTime, dt, avgDt, maxDt;

short loopCount = 0;

void setup() {
    Serial.begin(115200);
    pastTime = micros();

    imu.init();

    frontRight.init(5);
    backRight.init(6);
    backLeft.init(10);
    frontLeft.init(11);

    state.ARMED = false;
}

void loop() {

    imu.getDMPData(&meas);

    dt = micros() - pastTime;
    pastTime = micros();

    //if(imu.dataUpdated) {
    //    avgDt /= loopCount;

        //Serial.print(avgDt);
        //Serial.print(" , ");
        //Serial.print(maxDt);
        //Serial.print(", ");
        //Serial.println(Throttle.getCommand());
        //Serial.print(" , ");
        //Serial.println(Throttle.TIMEOUT);

    //    maxDt = 0;
    //    loopCount = 0;
    //} else {
    //    ++loopCount;
    //   avgDt += dt;
    //    maxDt = max(maxDt, dt);
    //}

    int throttle    = Throttle.getCommand();
    int roll        = Roll.getCommand();
    int pitch       = Pitch.getCommand();
    int yaw         = Yaw.getCommand();
    
    state.MODE = SwitchOne.getSwitchState();

    if(state.MODE == DISARMED || Throttle.TIMEOUT == true || Roll.TIMEOUT == true || Pitch.TIMEOUT == true || Yaw.TIMEOUT == true || SwitchOne.TIMEOUT == true || SwitchTwo.TIMEOUT == true) {
        state.ARMED = false;
    } 
    else state.ARMED = true;

    Serial.print(meas.rates[0]);
    Serial.print(" - ");
    Serial.print(meas.rates[1]);
    Serial.print(" - ");
    Serial.print(meas.rates[2]);
    Serial.print(" - ");
    Serial.print(meas.ypr[0]);
    Serial.print(" - ");
    Serial.print(meas.ypr[1]);
    Serial.print(" - ");
    Serial.println(meas.ypr[2]);

    frontRight.spin(throttle, state.ARMED);
    frontLeft.spin(throttle, state.ARMED);
    backLeft.spin(throttle, state.ARMED);
    backRight.spin(throttle, state.ARMED);
}