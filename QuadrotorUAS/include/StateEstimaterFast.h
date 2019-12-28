#ifndef STATE_ESTIM_FAST_H
#define STATE_ESTIM_FAST_H

class Kalman {
    public:
        Kalman();

        void init(float * init_error_meas, float * init_state);
        void computeState(float * meas, float * control, float * output);

    private:
        float combine_mean(float mean1, float variance1, float mean2, float variance2);
        float combine_var(float mean1, float variance1, float mean2, float variance2);
        void predict(float * output_ptr, float mean1, float variance1, float mean2, float variance2);
        void updatePrediction(float dt, float * u);
        void updateProcessNoise(void);
        void updateKalmanGain(void);
        void updateMeasurement(float * meas);
        void forwardProcessNoise(void);
        void updateStatePrediction(void);

        void covariance(float * var, float addr[6][6]);
        
        float dt = 0;
        float error_obs_meas[6], error_est_meas[6];

        float X[6];                         // State matrix
        float K[6][6];                         // Kalman Gain
        float Y[6];                         // Measurements

        float A[6][6];                   // State mapping
        float B[6][3];                   // Command mapping

        float P[6][6];      // Process noise
        float R[6][6];      // Measurement Covariance

        float W_k[6];                        // State noise
        float Q_t[6];                        // Noise in process noise
        float Z_m[6];                        // Measurement noise

        unsigned long prevTime;
};

#endif