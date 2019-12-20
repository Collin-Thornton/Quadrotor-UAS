#include "IMU.h"

#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;

IMU::IMU() {}

int IMU::init(void) {
    instance = this;

    Wire.begin();
    Wire.setClock(400000);

    Serial.println(F("Initalizing MPU6050..."));
    mpu.initialize();

    pinMode(3, INPUT);
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);

    Serial.println(F("Testing device connection..."));
    if(mpu.testConnection()) Serial.println(F("MPU6050 connection successful"));
    else {
        Serial.println(F("MPU6050 connection unsuccessful"));
        return 0;
    }

    Serial.println(F("Initializing DMP"));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if(devStatus == 0) {
        Serial.println(F("Enabling DMP"));
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(3), dmpISR, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP Ready. Waiting for first interrupt"));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else {
        Serial.print(F("DMP Initalization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return 0;
    }

    return 1;
}

void IMU::getDMPData(Meas * meas) {
    detachInterrupt(digitalPinToInterrupt(3));
    dmpReady = mpuInterrupt;
    mpuInterrupt = false;
    attachInterrupt(digitalPinToInterrupt(3), dmpISR, RISING);

    if(!dmpReady && fifoCount < packetSize) {
        dataUpdated = false;
        return;
    }

    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if((mpuIntStatus & 0x10 || fifoCount == 1024)) {
        mpu.resetFIFO();
        Serial.println(F("MPU6050 FIFO Overflow"));
    }
    else if(mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetGyro(meas->rates, fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(meas->ypr, &q, &gravity);

        meas->ypr[0] = -meas->ypr[0] * 180 / M_PI;
        meas->ypr[1] = -meas->ypr[1] * 180 / M_PI + 3.5;
        meas->ypr[2] =  meas->ypr[2] * 180 / M_PI - 1.9;

        long temp = meas->rates[0];
        meas->rates[0] = meas->rates[2] / 2000 / 25;
        meas->rates[1] = meas->rates[1] / 2000 / 25;
        meas->rates[2] = temp / 2000 / 25;

        dataUpdated = true;
    }
    return;
}

void IMU::dmpISR(void) {
    instance->mpuInterrupt = 
    true;    
}

