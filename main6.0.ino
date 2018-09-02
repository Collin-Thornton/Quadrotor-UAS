
#define EI_ARDUINO_INTERRUPTED_PIN
//#include <NeoSWSerial.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <EnableInterrupt.h>
#include <Servo.h>

#define MOTOR_PIN_5 5
#define MOTOR_PIN_6 6
#define MOTOR_PIN_10 10
#define MOTOR_PIN_11 11
#define INTERRUPT_PIN 3

//NeoSWSerial portGPS(15, 16);
Servo motor5, motor6, motor10, motor11;
MPU6050 mpu;

float rollVal, pitchVal, yawVal, rollSpeed, pitchSpeed, yawSpeed;
float pastRollVal  = 0, pastPitchVal = 0, pastYawVal = 0, pastTime = 0, dt, ROTATIONFACTOR = 7;
float P[2] = {3.0, 3.0}, I[2] = {2.0, 2.0}, rateP[3] = {1.65, 1.65, 8}, rateI[3] = {2.95, 2.95, 2};
float integralError[2] = {0, 0}, rateIntegralError[3] = {0, 0, 0}, angleOutputMultiplier[2] = {2, 2};
short x = 0, angleModeCounter = 0, voltCounter = 0;
bool switchOnePast = false, switchTwoPast = false, ANGLE_MODE = false, ARMED = false, VOLT_ALARM = true, SERIAL_MONITOR = false;

volatile short throttleShared, rollShared, yawShared, pitchShared, switchOneShared, switchTwoShared;
volatile int timer0, timer1, timer2, timer3, timer4, timer5;

bool dmpReady = false, dataUpdated = true, doCalibrationWait = false;
uint8_t mpuIntStatus, devStatus, fifoBuffer[64];
uint16_t packetSize, fifoCount;

Quaternion q;
VectorInt16 aa, aaReal, aaWorld;
VectorFloat gravity;
float euler[3], ypr[3];

//-----------------------------------|Beginning of Interrupt Functions for PWM Input|----------------------------//

//void portGPSISR() {
//  NeoSWSerial::rxISR( *portInputRegister( digitalPinToPort( 15 )));
//}

void throttleInterrupt() {
  if (arduinoPinState != 0)
    timer0 = micros();
  else {
    if (timer0 != 0) {
      throttleShared = ((volatile unsigned int)micros() - timer0);
      timer0 = 0;
    }
  }
}
void rollInterrupt() {
  if (arduinoPinState != 0)
    timer1 = micros();
  else {
    if (timer1 != 0) {
      rollShared = ((volatile unsigned int)micros() - timer1);
      timer1 = 0;
    }
  }
}
void yawInterrupt() {
  if (arduinoPinState != 0)
    timer2 = micros();
  else {
    if (timer2 != 0) {
      yawShared = ((volatile unsigned int)micros() - timer2);
      timer2 = 0;
    }
  }
}
void pitchInterrupt() {
  if (arduinoPinState != 0)
    timer3 = micros();
  else {
    if (timer3 != 0) {
      pitchShared = ((volatile unsigned int)micros() - timer3);
      timer3 = 0;
    }
  }
}
void switchOneInterrupt() {
  if (arduinoPinState != 0)
    timer4 = micros();
  else {
    if (timer4 != 0) {
      switchOneShared = ((volatile unsigned int)micros() - timer4);
      timer4 = 0;
    }
  }
}
void switchTwoInterrupt() {
  if (arduinoPinState != 0)
    timer5 = micros();
  else {
    if (timer5 != 0) {
      switchTwoShared = ((volatile unsigned int)micros() - timer5);
      timer5 = 0;
    }
  }
}

//----------------------------------|End of Interrupt Functions|------------------------------------------------------//
//----------------------------------|Interrupt Function for MPU6050|------------------------------------//

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

//--------------------------------|End MPU6050 Interrupt Function|--------------------------------------------//
float stringToFloat(char BTC[8]) { // Converts string input to float (first used in line 210)
  short a = 2, y = 0;
  char cache[6] = {0, 0, 0, 0, 0};
  while (a < x) {
    cache[y] = BTC[a];
    a++;
    y++;
    if (a == x) {
      return (atof(cache));
    }
  }
}

float rateToFloat(char BTC[9]) {
  short a = 3, y = 0;
  char cache[6] = {0, 0, 0, 0, 0, 0};
  while (a < x) {
    cache[y] = BTC[a];
    a++;
    y++;
    if (a == x) return (atof(cache));
  }
}

void setup() {

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  //  portGPS.begin(38400);
  //  enableInterrupt(15, portGPSISR, CHANGE);

  Serial.println(F("Initializing MPU6050... "));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing devcice connections... "));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP... "));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP... "));
    mpu.setDMPEnabled(true);
    enableInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP Ready! Waiting for first interrupt... "));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();

  }
  else {
    Serial.print(F("DMP Initialization faile (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(7, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  enableInterrupt(7, throttleInterrupt, CHANGE);
  enableInterrupt(2, rollInterrupt, CHANGE);
  enableInterrupt(8, yawInterrupt, CHANGE);
  enableInterrupt(4, pitchInterrupt, CHANGE);
  enableInterrupt(14, switchOneInterrupt, CHANGE);
  enableInterrupt(13, switchTwoInterrupt, CHANGE);

  motor5.attach(MOTOR_PIN_5);
  motor6.attach(MOTOR_PIN_6);
  motor10.attach(MOTOR_PIN_10);
  motor11.attach(MOTOR_PIN_11);

  motor5.writeMicroseconds(1000);
  motor6.writeMicroseconds(1000);
  motor10.writeMicroseconds(1000);
  motor11.writeMicroseconds(1000);

  Serial.println(F("Waiting for MPU6050 calibration to complete..."));
  uint8_t startUpTime = millis();

  while (millis() - startUpTime < 24000 && doCalibrationWait == true) {      // Wait 25 seconds to complete MPU6050 DMP calibration.
    if ((millis() - startUpTime) % 1000 == 0) tone(9, 1500);
    else if ((millis() - startUpTime - 500) % 1000 == 0) noTone(9);
  }

  for (uint8_t toneCount = 0; toneCount < 3; toneCount++) {
    tone(9, 1500);
    delay(250);
    noTone(9);
    delay(250);
  }

}

void loop() {

  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {

    while ((micros() - pastTime) / 1000000 < .0106 && mpuInterrupt == false) { }
    if (mpuInterrupt == true) break;

    static short throttle, roll, pitch, yaw, switchOne, switchTwo;

    noInterrupts();
    throttle = throttleShared;
    roll = rollShared;
    pitch = pitchShared;
    yaw = yawShared;
    switchOne = switchOneShared;
    switchTwo = switchTwoShared;
    interrupts();

    float batteryVoltage = ((analogRead(3) - 56) * .00488 * 4.67) - .55;
    if (batteryVoltage <= 11.0 && VOLT_ALARM == true) { // Voltage calculator for the LiPo - hooked up to a voltage divider circuit that converts 12v DC to about 4.5V DC
      tone(9, 1600);
      voltCounter = 0;
    }
    else {
      voltCounter++;
      if (voltCounter > 100) {
        noTone(9);
        voltCounter = 0;
      }
    }

    //reset integral buildup on change of motor ARM status//
    if (switchTwo < 1500) { // Mechanism to change ARMED status of motors//
      if (switchTwoPast == false) Serial.println(F("----|UNARMED|----"));
      switchTwoPast = true;
      ARMED = false;
    }
    else ARMED = true;
    if (switchTwoPast == true &&  switchTwo > 1500) {
      for (short t = 0; t < 3; t++) {
        integralError[t] = 0;
        rateIntegralError[t] = 0;
      }
      Serial.print(F("Integral reset to "));
      Serial.print(integralError[0]); // Reset integral error buildup to 0 on activation of kill switch
      Serial.print(F(", "));
      Serial.print(integralError[1]);
      Serial.print(F(". "));
      Serial.print(F("Rate Integral: "));
      Serial.print(rateIntegralError[0]);
      Serial.print(F(", "));
      Serial.print(rateIntegralError[1]);
      Serial.print(F(", "));
      Serial.print(rateIntegralError[2]);
      Serial.println(F("."));
      Serial.println(F("----|ARMED|----"));
      switchTwoPast = false;
    }

    if (switchOne < 1500 && switchOnePast == false) {
      ANGLE_MODE = false;
      switchOnePast = true;
      Serial.println(F("----|ACRO_MODE|----"));
    }
    else if (switchOne >= 1500 && switchOnePast == true) {
      ANGLE_MODE = true;
      switchOnePast = false;
      Serial.println(F("----|ANGLE_MODE|----"));
    }


    //keep accurate time of loop speed//
    float dt = (micros()  - pastTime) / 1000000;
    pastTime = micros();

    //---------------------------------------|signal processing|------------------------------------//
    //algorithm for compensation of yaw rollover//
    //    if(dataUpdated == true && abs(yawVal - pastYawVal) > .05){
    if (abs(yawVal - pastYawVal) > .01) {
      if (yawVal < -170 && pastYawVal > 170) {
        float f, g;
        f = -180 - yawVal;
        g = 180 - pastYawVal;
        yawSpeed = ((f - g) / dt) - 1.3;
      }
      else if (yawVal > 170 && pastYawVal < -170) {
        float f, g;
        f = 180 - yawVal;
        g = -180 - pastYawVal;
        yawSpeed = ((f - g) / dt) - 1.3;
      }
      else yawSpeed = ((yawVal - pastYawVal) / dt) - 1.3;

      pastYawVal = yawVal;
    }
    //calculate rotational speed as derivative of Euler Angles//
    if (abs(rollVal - pastRollVal) > .01) {
      rollSpeed = (rollVal - pastRollVal) / dt;
      pastRollVal = rollVal;
    }

    if (abs(pitchVal - pastPitchVal) > .01) {
      pitchSpeed = (pitchVal - pastPitchVal) / dt;
      pastPitchVal = pitchVal;
    }

    //----------------------------------|end of signal processing|---------------------------------------------------------//
    //--------------------------------------|bluetooth processing|------------------------------------------------------------//

    char BTC[9];
    //switch case menu for quickly parsing bluetooth data character by character//
    if (Serial.available()) { // Interpret serial data sent from a Microsoft Visual Studio GUI coded in C#, used to wireless change PI values and reset integral buildup. Will eventually be used to monitor attitude, position, etc.
      BTC[x] = Serial.read();
      if (BTC[x] == '\n') {
        for (short b = 0; b <= x; b++) {
          Serial.write(BTC[b]);
        }

        switch (BTC[0]) {
          case 'A':
            switch (BTC[1]) {
              case 'M':
                switch (BTC[2]) {
                  case 'R':
                    angleOutputMultiplier[0] = rateToFloat(BTC);
                    Serial.print(F("angleOutputMultiplier[0] set to: "));
                    Serial.println(angleOutputMultiplier[0]);
                    break;
                  case 'P':
                    angleOutputMultiplier[1] = rateToFloat(BTC);
                    Serial.print(F("angleOutputMultiplier[1] set to: "));
                    Serial.println(angleOutputMultiplier[1]);
                    break;
                }
                break;
              case 'N':
                switch (BTC[2]) {
                  case 'G':
                    if (ANGLE_MODE == true) {
                      ANGLE_MODE = false;
                      Serial.println(F("ACRO_MODE"));
                    }
                    else {
                      ANGLE_MODE = true;
                      Serial.println(F("ANGLE_MODE"));
                    }
                    break;
                }
                break;
              case 'R':
                switch (BTC[2]) {
                  case 'P':
                    rateP[0] = rateToFloat(BTC);
                    Serial.print(F("rateRollP set to "));
                    Serial.println(rateP[0]);
                    break;
                  case 'I':
                    rateI[0] = rateToFloat(BTC);
                    Serial.print(F("rateRollI set to "));
                    Serial.println(rateI[0]);
                    break;
                }
                break;
              case 'P':
                switch (BTC[2]) {
                  case 'P':
                    rateP[1] = rateToFloat(BTC);
                    Serial.print(F("ratePitchP set to "));
                    Serial.println(rateP[1]);
                    break;
                  case 'I':
                    rateI[1] = rateToFloat(BTC);
                    Serial.print(F("ratePitchI set to "));
                    Serial.println(rateI[1]);
                    break;
                }
                break;
              case 'Y':
                switch (BTC[2]) {
                  case 'P':
                    rateP[2] = rateToFloat(BTC);
                    Serial.print(F("rateYawP set to "));
                    Serial.println(rateP[2]);
                    break;
                  case 'I':
                    rateI[2] = rateToFloat(BTC);
                    Serial.println(F("rateYawI set to "));
                    Serial.println(rateI[2]);
                    break;
                }
                break;
            }
            break;
          case 'R':
            switch (BTC[1]) {
              case 'P':
                P[0] = stringToFloat(BTC);
                Serial.print(F("rollP set to "));
                Serial.println(P[0]);
                break;
              case 'I':
                I[0] = stringToFloat(BTC);
                Serial.print(F("rollI set to "));
                Serial.println(I[0]);
                break;
              case 'O':
                switch (BTC[2]) {
                  case 'T':
                    char rotateBuffer[3] = {0, 0, 0};
                    for (int p = 0; p < 3; p++) {
                      rotateBuffer[p] = BTC[p + 3];
                    }
                    ROTATIONFACTOR = atoi(rotateBuffer);
                    Serial.print(F("ROTATIONFACTOR set to "));
                    Serial.println(ROTATIONFACTOR);
                    break;
                }
                break;
            }
            break;
          case 'P':
            switch (BTC[1]) {
              case 'P':
                P[1] = stringToFloat(BTC);
                Serial.print(F("pitchP set to "));
                Serial.println(P[1]);
                break;
              case 'I':
                I[1] = stringToFloat(BTC);
                Serial.print(F("pitchI set to "));
                Serial.println(I[1]);
                break;
            }
            break;
          case 'Y':
            switch (BTC[1]) {
              case 'P':
                P[2] = stringToFloat(BTC);
                Serial.print(F("yawP set to "));
                Serial.println(P[2]);
                break;
              case 'I':
                I[2] = stringToFloat(BTC);
                Serial.print(F("yawI set to "));
                Serial.println(I[2]);
                break;
            }
            break;
          case 'I':
            switch (BTC[1]) {
              case 'R':
                for (short t = 0; t < 3; t++) {
                  integralError[t] = 0;
                }

                Serial.print(F("Integral error reset to: "));
                Serial.print(integralError[0]);
                Serial.print(F(", "));
                Serial.print(integralError[1]);
                Serial.print(F(", "));
                Serial.println(integralError[2]);
                break;
            }
            break;
          case 'B':
            switch (BTC[1]) {
              case 'A':
                switch (BTC[2]) {
                  case 'T':
                    Serial.println(batteryVoltage, 3);
                    break;
                  case 'O':
                    if (VOLT_ALARM == true) VOLT_ALARM = false;
                    else VOLT_ALARM = true;
                    Serial.print(F("VOLT_ALARM = "));
                    Serial.println(VOLT_ALARM);
                    break;
                }
                break;
            }
          case 'M':
            switch (BTC[1]) {
              case 'O':
                switch (BTC[2]) {
                  case 'N':
                    if (SERIAL_MONITOR == false) {
                      SERIAL_MONITOR = true;
                      Serial.print(F("Serial Monitor = "));
                      Serial.println(SERIAL_MONITOR);
                    }
                    else {
                      SERIAL_MONITOR = false;
                      Serial.print(F("Serial Monitor = "));
                      Serial.println(SERIAL_MONITOR);
                    }
                }
            }
            break;
        }
        x = -1;
      }
      x++;
    }

    //----------------------------------------------|end bluetooth processing|----------------------------------------------//
    //----------------------------------------------|control mechanism|------------------------------------------------------//
    //cascading PI controller for control. If ANGLE_MODE is disabled, only one controller per axis. If it is enabled, there are two//
    short scaledYawCommand, scaledRollCommand, scaledPitchCommand, rateRollCommand, ratePitchCommand, TILTFACTOR = 16;
    float rollError, pitchError, rateRollError, ratePitchError, rateYawError, angleOutput[2];

    //---|rate loop|---//
    if (ANGLE_MODE == false) {
      rateRollError = ((roll - 1500) / ROTATIONFACTOR) - rollSpeed;
      rateIntegralError[0] += rateRollError * dt;
      scaledRollCommand = rateP[0] * rateRollError + rateI[0] * rateIntegralError[0];

      ratePitchError = ((pitch - 1500) / ROTATIONFACTOR) - pitchSpeed;
      rateIntegralError[1] += ratePitchError * dt;
      scaledPitchCommand = rateP[1] * ratePitchError + rateI[1] * rateIntegralError[1];

      rateYawError = ((1500 - yaw) / (ROTATIONFACTOR / 4)) - yawSpeed;
      rateIntegralError[2] += rateYawError * dt;
      scaledYawCommand = rateP[2] * rateYawError + rateI[2] * rateIntegralError[2];
    }

    //---|end rate loop|---//
    //---|angle loop|---//
    else {


      rollError = ((roll - 1500) / TILTFACTOR) - rollVal;
      integralError[0] += rollError * dt;

      pitchError = ((pitch - 1500) / TILTFACTOR) - pitchVal;
      integralError[1] += pitchError * dt;

      if (angleModeCounter == 3) { // angle loop consists of a rate loop nested within a master angle loop//

        angleOutput[0] = P[0] * rollError + I[0] * integralError[0];
        angleOutput[1] = P[1] * pitchError + I[1] * integralError[1];

        angleModeCounter = 0;
      }
      angleModeCounter++;

      //rate loop//
      rateRollError = (angleOutput[0] * angleOutputMultiplier[0]) - rollSpeed;
      rateIntegralError[0] += rateRollError * dt;
      scaledRollCommand = rateP[0] * rateRollError + rateI[0] * rateIntegralError[0];

      ratePitchError = (angleOutput[1] * angleOutputMultiplier[1]) - pitchSpeed;
      rateIntegralError[1] += ratePitchError * dt;
      scaledPitchCommand = rateP[1] * ratePitchError + rateI[1] * rateIntegralError[1];

      rateYawError = ((1500 - yaw) / (ROTATIONFACTOR / 4)) - yawSpeed;
      rateIntegralError[2] += rateYawError * dt;
      scaledYawCommand = rateP[2] * rateYawError + rateI[2] * rateIntegralError[2];
      //angle loop//

    }

    //---|end angle loop|---//

    if (throttle > 1900) throttle = 1900;

    short motor5Speed = throttle - (scaledYawCommand / 2) - (scaledRollCommand / 2) - (scaledPitchCommand / 2);
    if (motor5Speed > 2000) motor5Speed = 2000;
    else if (motor5Speed < 1060) motor5Speed = 1060;
    short motor6Speed = throttle + (scaledYawCommand / 2) - (scaledRollCommand / 2) + (scaledPitchCommand / 2); // Addition and scaling of motor values for all three axises.
    if (motor6Speed > 2000) motor6Speed = 2000;
    else if (motor6Speed < 1060) motor6Speed = 1060;
    short motor10Speed = throttle - (scaledYawCommand / 2) + (scaledRollCommand / 2) + (scaledPitchCommand / 2);
    if (motor10Speed > 2000) motor10Speed = 2000;
    else if (motor10Speed < 1060) motor10Speed = 1060;
    short motor11Speed = throttle + (scaledYawCommand / 2) + (scaledRollCommand / 2) - (scaledPitchCommand / 2);
    if (motor11Speed > 2000) motor11Speed = 2000;
    else if (motor11Speed < 1060) motor11Speed = 1060;

    if (ARMED == true) {
      motor5.writeMicroseconds(motor5Speed);
      motor6.writeMicroseconds(motor6Speed);
      motor10.writeMicroseconds(motor10Speed); // Update PWM signal to motors.
      motor11.writeMicroseconds(motor11Speed);
    }
    else {
      motor5.writeMicroseconds(1000);
      motor6.writeMicroseconds(1000);
      motor10.writeMicroseconds(1000);
      motor11.writeMicroseconds(1000);
    }
    //-----------------------------|end of control mechanism|--------------------------------------------------//


    if (SERIAL_MONITOR == true) {
      Serial.print(rollVal);
      Serial.print(F(","));
      Serial.print(pitchVal);
      Serial.print(F(","));
      Serial.print(yawVal);
      Serial.print(F(","));
      Serial.print(motor5Speed);
      Serial.print(F(","));
      Serial.print(motor6Speed);
      Serial.print(F(","));
      Serial.print(motor10Speed);
      Serial.print(F(","));
      Serial.print(motor11Speed);
      Serial.print(F(","));
      Serial.print(scaledRollCommand);
      Serial.print(F(","));
      Serial.print(scaledPitchCommand);
      Serial.print(F(","));
      Serial.print(scaledYawCommand);
      Serial.print(F(","));
      Serial.println(dt, 4);
    }

  }

  //----------------------------------------------------------|handle new data from MPU6050|---------------------------------------//

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO Overflow!"));
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    pitchVal = ((-1 * ypr[2]) * 180 / M_PI) - .05;
    rollVal = ((-1 * ypr[1]) * 180 / M_PI) + 7.08;
    yawVal = -1 * ypr[0] * 180 / M_PI;
    dataUpdated = true;

  }

}
