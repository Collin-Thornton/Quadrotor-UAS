#include "StateEstimaterFast.h"
#include <Arduino.h>

Kalman::Kalman() {}

void Kalman::init(float * init_error_meas, float * init_state) {
    for(int i=0; i<6; ++i) {
        error_est_meas[i] = init_error_meas[i];
        W_k[i] = 0;
        Q_t[i] = 0;
        Z_m[i] = 0;         // should be variance of measurements

        X[0] = init_state[0];
    }
    
    covariance(error_est_meas, P);
    prevTime = micros();
}
void Kalman::computeState(float * meas, float * control, float * output) {
    dt = (float)((micros() - prevTime)/1000000.0);
    prevTime = micros();

    int n = 1;//sizeof(meas)/sizeof(meas[0]);

    for(int i=0; i<n; ++i) {
        for(int j=0; j<3; ++j) X[j] += X[j+3]*dt + W_k[j];
        for(int j=3; j<6; ++j) X[j] += control[i]*.5*dt*dt;
        
        updatePrediction(dt, control);
        updateProcessNoise();
        updateKalmanGain();
        updateMeasurement(meas);
        forwardProcessNoise();
        //updateStatePrediction();
    }

    for(int i=0; i<6; ++i) {
        output[i] = X(i);
    }

    return;
}

void Kalman::updatePrediction(float dt, float * u) {
    A <<    1 ,0, 0, dt, 0 ,0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

    B <<    0.5*dt*dt, 0, 0,
            0, 0.5*dt*dt, 0,
            0, 0, 0.5*dt*dt,
            dt,    0,    0,
            0,     dt,   0,
            0,     0,   dt;

    BLA::Matrix<3,1> U = {  u[0],
                            u[1],
                            u[2] };

    X = A*X;
    X += B*U;
    X += W_k;
    return;
}
void Kalman::updateProcessNoise(void) {
    P = A*P;
    P *= ~A;
    P += Q_t;
    return;
}
void Kalman::updateKalmanGain(void) {
    covariance(error_obs_meas, R.Ref());
    S = H*P;
    S *= ~H ;
    S += R;

    K = (P*~H);
    BLA::Invert(S);
    //K *= S;   //(H*P*~H+R).Inverse();  //S.Inverse();
    //Serial << K;
    //Serial.println(' ');
    return;
}
void Kalman::updateMeasurement(float * meas) {
    Y <<    meas[0],
            meas[1],
            meas[2],
            meas[3],
            meas[4],
            meas[5];

    Y = H*Y + Z_m;  // Z_m should be variance of measurements
    return;
}
void Kalman::forwardProcessNoise(void) {
    P = (H - K*H);//*P;  // UNCOMMENT THIS!!!!!!!!!!!!
    return;
}
void Kalman::updateStatePrediction(void) {
    X = X + K*(Y - H*X);

    return;
}


float Kalman::combine_mean(float mean1, float variance1, float mean2, float variance2) {
    float output = (variance2*mean1 + variance1*mean2) / (variance2 + variance1);
    return output;
}
float Kalman::combine_var(float mean1, float variance1, float mean2, float variance2) {
    float output = 1 / ((1/variance2) + (1/variance1));
    return output;
}
void Kalman::predict(float * output_ptr, float mean1, float variance1, float mean2, float variance2) {
    output_ptr[0] = mean1 + mean2;
    output_ptr[1] = variance1 + variance2;
    return;
}
void Kalman::covariance(float * var, BLA::RefMatrix<6,6,Diagonal<6,float>> addr) {  
    for(int i=0; i<6; ++i) addr(i,i) = var[i]*var[i];
    return;
}