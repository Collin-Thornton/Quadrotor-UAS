#ifndef STATE_ESTIM_H
#define STATE_ESTIM_H

class Kalman {
    public:
        Kalman();

        float * computeState(float *ypr_meas);

    private:
        

        float* prevState;

};

#endif