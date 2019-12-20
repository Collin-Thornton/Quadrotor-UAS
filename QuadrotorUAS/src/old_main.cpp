/*#include <Arduino.h>



//#include <NeoSWSerial.h>


float rollVal, pitchVal, yawVal, rollSpeed, pitchSpeed, yawSpeed;
float pastRollVal  = 0, pastPitchVal = 0, pastYawVal = 0, pastTime = 0, dt, ROTATIONFACTOR = 7;
float P[2] = {3.0, 3.0}, I[2] = {2.0, 2.0}, rateP[3] = {1.65, 1.65, 8}, rateI[3] = {2.95, 2.95, 2};
float integralError[2] = {0, 0}, rateIntegralError[3] = {0, 0, 0}, angleOutputMultiplier[2] = {2, 2};
short x = 0, angleModeCounter = 0, voltCounter = 0;
bool switchOnePast = false, switchTwoPast = false, ANGLE_MODE = false, ARMED = false, VOLT_ALARM = true, SERIAL_MONITOR = false;

//-----------------------------------|Beginning of Interrupt Functions for PWM Input|----------------------------//

//void portGPSISR() {
//  NeoSWSerial::rxISR( *portInputRegister( digitalPinToPort( 15 )));
//}



void setup() {

  Serial.begin(115200);
  //  portGPS.begin(38400);
  //  enableInterrupt(15, portGPSISR, CHANGE);


  pinMode(9, OUTPUT);
  

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



  }

}*/