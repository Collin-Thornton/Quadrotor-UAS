#include "IMU.h"

#include <Wire.h>
#include <EEPROM.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;

IMU::IMU() {}

int IMU::init(bool calibrate) {
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

    if(calibrate) {
       Serial.println(F("Calibrating MPU6050"));
       mpu.CalibrateAccel(100);
       mpu.CalibrateGyro(100);

       EEPROM.put(0, mpu.getXGyroOffset());
       EEPROM.put(0+1*sizeof(int16_t), mpu.getYGyroOffset());
       EEPROM.put(0+2*sizeof(int16_t), mpu.getZGyroOffset());
       EEPROM.put(0+3*sizeof(int16_t), mpu.getXAccelOffset());
       EEPROM.put(0+4*sizeof(int16_t), mpu.getYAccelOffset());
       EEPROM.put(0+5*sizeof(int16_t), mpu.getZAccelOffset());

       Serial.print(mpu.getXGyroOffset());
       Serial.print('\t');
       Serial.print(mpu.getYGyroOffset());
       Serial.print('\t');
       Serial.print(mpu.getZGyroOffset());
       Serial.print('\t');
       Serial.print(mpu.getXAccelOffset());
       Serial.print('\t');
       Serial.print(mpu.getYAccelOffset());
       Serial.print('\t');
       Serial.println(mpu.getZAccelOffset());
       delay(5000);
    } else {
        int16_t gyroXoff, gyroYoff, gyroZoff, accelXoff, accelYoff, accelZoff;

        EEPROM.get(0, gyroXoff);
        EEPROM.get(0+1*sizeof(int16_t), gyroYoff);
        EEPROM.get(0+2*sizeof(int16_t), gyroZoff);
        EEPROM.get(0+3*sizeof(int16_t), accelXoff);
        EEPROM.get(0+4*sizeof(int16_t), accelYoff);
        EEPROM.get(0+5*sizeof(int16_t), accelZoff);

        mpu.setXGyroOffset(gyroXoff);
        mpu.setYGyroOffset(gyroYoff);
        mpu.setZGyroOffset(gyroZoff);
        mpu.setXAccelOffset(accelXoff);
        mpu.setYAccelOffset(accelYoff);
        mpu.setZAccelOffset(accelZoff);
    }

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

void IMU::getDMPData(float * output) {
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

        float ypr[3] = {0.0, 0.0, 0.0};
        int rates[3] = {0, 0, 0};

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGyro(rates, fifoBuffer);

        ypr[0] *= -180.0 / M_PI;
        ypr[1] *= -180.0 / M_PI + 3.6;
        ypr[2] *=  180.0 / M_PI - 1.9;

        int temp = rates[0];
        rates[0] = rates[2];
        rates[1] = rates[1];
        rates[2] = temp;

        output[0] = ypr[0];
        output[1] = ypr[1];
        output[2] = ypr[2];
        output[3] = (float)rates[0];
        output[4] = (float)rates[1];
        output[5] = (float)rates[2];

        dataUpdated = true;
    }
    return;
}

void IMU::dmpISR(void) {
    instance->mpuInterrupt = true;    
}

