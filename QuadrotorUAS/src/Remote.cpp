#include "Remote.h"
#include <EnableInterrupt.h>

Remote::Remote(short pin) {
    pinMode(pin, INPUT_PULLUP);
    this->pin = pin;
    update_time = micros();

    switch(pin) {
        case 7:
            myISR = static_ISR_throttle;
            enableInterrupt(7, myISR, CHANGE);
            instance[0] = this;
            break;
        case 2:
            myISR = static_ISR_roll;
            enableInterrupt(2, myISR, CHANGE);
            instance[1] = this;
            break;
        case 4:
            myISR = static_ISR_pitch;
            enableInterrupt(4, myISR, CHANGE);
            instance[2] = this;
            break;
        case 8:
            myISR = static_ISR_yaw;
            enableInterrupt(8, myISR, CHANGE);
            instance[3] = this;
            break;
        case 14:
            myISR = static_ISR_switchOne;
            enableInterrupt(14, myISR, CHANGE);
            instance[4] = this;
            break;
        case 13:
            myISR = static_ISR_switchTwo;
            enableInterrupt(13, myISR, CHANGE);
            instance[5] = this;
            break;
    }
}

int Remote::getCommand(void) {
    disableInterrupt(pin);
    int output = command;
    enableInterrupt(pin, myISR, CHANGE);

    if(!change) {
        if(millis() - update_time > 100) {
            TIMEOUT = true;
            return 1000;
        }
        return output;
    }

    update_time = millis();
    change = false;

    history[history_counter++] = output;
    if(history_counter > 4) history_counter = 0;

    if(history[0] == history[1] && history[1] == history[2] && history[2] == history[3] && history[3] == history[4]) {
        TIMEOUT = true;
        output = 1000;
        return output;
    }
   
    TIMEOUT = false;
    return output;
}

int Remote::getSwitchState(void) {
    if(instance[4] == NULL || instance[5] == NULL) return DISARMED;

    int switchOne = instance[4]->getCommand();
    int switchTwo = instance[5]->getCommand();

    if(switchOne < 1500 && switchTwo < 1500) return DISARMED;
    if(switchOne > 1500 && switchTwo < 1500) return RATE_MODE;
    if(switchOne < 1500 && switchTwo > 1500) return ANGLE_MODE;
    
    return DISARMED;
}

void Remote::dynamic_ISR(void) {
    if(arduinoPinState != 0) timer = (volatile unsigned int)micros();
    else {
        if (timer != 0) {
            command = ((volatile unsigned int)micros() - timer);
            timer = 0;
            change = true;
        }
    }
}


void Remote::static_ISR_throttle(void) {
    if(Remote::instance[0] != NULL) Remote::instance[0]->dynamic_ISR();
}
void Remote::static_ISR_roll(void) {
    if(Remote::instance[1] != NULL) Remote::instance[1]->dynamic_ISR();
}
void Remote::static_ISR_pitch(void) {
    if(Remote::instance[2] != NULL) Remote::instance[2]->dynamic_ISR();
}
void Remote::static_ISR_yaw(void) {
    if(Remote::instance[3] != NULL) Remote::instance[3]->dynamic_ISR();
}
void Remote::static_ISR_switchOne(void) {
    if(Remote::instance[4] != NULL) Remote::instance[4]->dynamic_ISR();
}
void Remote::static_ISR_switchTwo(void) {
    if(Remote::instance[5] != NULL) Remote::instance[5]->dynamic_ISR();
}