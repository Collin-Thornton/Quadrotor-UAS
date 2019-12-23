#ifndef IMU_H
#define IMU_H

// Class IMU declared in IMU.cpp due to conflictions in "MPU6050_6Axis_MotionApps20.h"

class IMU {
    public:
        IMU();

        int init(bool calibrate = true);
        void getDMPData(float * output); // ypr

        static IMU * instance;

        bool dataUpdated = false;

    private:
        static void dmpISR(void);

        volatile bool mpuInterrupt = false;
        bool dmpReady = false;

        unsigned char  mpuIntStatus, devStatus, fifoBuffer[64];
        unsigned int packetSize, fifoCount;
};

#endif