#ifndef IMU_H
#define IMU_H

// Class IMU declared in IMU.cpp due to conflictions in "MPU6050_6Axis_MotionApps20.h"

struct Meas {
    float * ypr;
    long * rates;
};

class IMU {
    public:
        IMU();

        int init(void);
        void getDMPData(Meas * meas); // ypr

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