#ifndef REMOTE_H
#define REMOTE_H

#define EI_ARDUINO_INTERRUPTED_PIN
#define EI_NOTEXTERNAL
#define DISARMED    0
#define RATE_MODE   1
#define ANGLE_MODE  2
//#define UNDEF    3

struct State {
    bool ARMED;
    int  MODE;
}; 

class Remote {
    public:
        Remote(short pin);

        int getCommand(void);
        int getSwitchState(void);

        static Remote * instance[6];
        bool TIMEOUT = true;

    private:
        void dynamic_ISR(void);

        static void static_ISR_throttle(void);
        static void static_ISR_roll(void);
        static void static_ISR_pitch(void);
        static void static_ISR_yaw(void);
        static void static_ISR_switchOne(void);
        static void static_ISR_switchTwo(void);

        volatile unsigned int command, timer;

        int update_time;//, avg_counter;
        //int history[67];

        short pin;
        void (*myISR)(void);
};

#endif