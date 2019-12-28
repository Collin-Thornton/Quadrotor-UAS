#include <Arduino.h>

//#include "SerialInput.h"
#include "PID.h"
#include "Motor.h"
#include "IMU.h"
#include "Remote.h"
#include "StateEstimaterFast.h"

Motor frontRight, backRight, backLeft, frontLeft;

Remote * Remote::instance[6];
Remote Throttle(7), Roll(2), Pitch(4), Yaw(8), SwitchOne(14), SwitchTwo(13);
State state; 

IMU * IMU::instance;
IMU imu;
float meas[6] = {0,0,0,0,0,0};

Kalman KF;
 
long pastTime, dt, avgDt, maxDt;

short loopCount = 0;

float control[3] = {0,0,0}, kf[6] = {0,0,0,0,0,0};

void setup() {
/*    Serial.begin(38400);
    char * response;
    char input;
    Serial.print("AT\r\n");
    int time = millis();
    while(millis() - time < 3000 && input != '\n') {
        if(Serial.available()) {
            input = Serial.read();
            response += input;
        }
    }
    delay(1000);
    Serial.print("AT+UART=115200,1,0\r\n");
    delay(1000);
    Serial.print("AT+RESET\r\n");
    delay(5000);
*/
    Serial.begin(115200);
    pastTime = micros();

    imu.init(false);

    frontRight.init(5);
    backRight.init(6);
    backLeft.init(10);
    frontLeft.init(11);

    state.ARMED = false;

    float error_meas[6] = {0, 0, 0, 0, 0, 0};
    imu.getDMPData(meas);
    KF.init(error_meas, meas);
}

void loop() {
    imu.getDMPData(meas);

    dt = micros() - pastTime;
    pastTime = micros();

    Serial.println(dt);

    int throttle    = Throttle.getCommand();
    int roll        = Roll.getCommand();
    int pitch       = Pitch.getCommand();
    int yaw         = Yaw.getCommand();
    
    state.MODE = SwitchOne.getSwitchState();

    if(state.MODE == DISARMED || Throttle.TIMEOUT == true || Roll.TIMEOUT == true || Pitch.TIMEOUT == true || Yaw.TIMEOUT == true || SwitchOne.TIMEOUT == true || SwitchTwo.TIMEOUT == true) {
        state.ARMED = false;
    } 
    else state.ARMED = true;


    if(imu.dataUpdated) {
        imu.dataUpdated = false;
        avgDt /= loopCount;

        maxDt = 0;
        loopCount = 0;

        KF.computeState(meas, control, kf);
        
        for(int i=0; i<6; ++i) {
            //Serial.print(meas[i]);
            //Serial.print(F(" , "));
            //Serial.print(kf[i]);
            //Serial.print(F("\t"));
        }
        Serial.println(' ');        
    } else {       
        ++loopCount;
        avgDt += dt;
        maxDt = max(maxDt, dt);
    }

    frontRight.spin(throttle, state.ARMED);
    frontLeft.spin(throttle, state.ARMED);
    backLeft.spin(throttle, state.ARMED);
    backRight.spin(throttle, state.ARMED);
}