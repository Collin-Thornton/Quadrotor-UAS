#ifndef STATE_ESTIM_H
#define STATE_ESTIM_H

#include <BasicLinearAlgebra.h>

class Kalman {
    public:
        Kalman();

        void init(float * init_error_meas, float * init_state);
        void computeState(float * meas, float * control, float * output);

    private:
        template<short dim, class ElemT> struct Diagonal {
            mutable ElemT m[dim];
            typedef ElemT elem_t;

            ElemT &operator()(short row, short col) const {
                static ElemT zero;
                if(row == col && row < dim) return m[row];
                else return (zero = 0);
            }
        }; 

        float combine_mean(float mean1, float variance1, float mean2, float variance2);
        float combine_var(float mean1, float variance1, float mean2, float variance2);
        void predict(float * output_ptr, float mean1, float variance1, float mean2, float variance2);
        void updatePrediction(float dt, float * u);
        void updateProcessNoise(void);
        void updateKalmanGain(void);
        void updateMeasurement(float * meas);
        void forwardProcessNoise(void);
        void updateStatePrediction(void);

        void covariance(float * var, BLA::RefMatrix<6,6,Diagonal<6,float>> addr);
        
        float dt = 0;
        float error_obs_meas[6], error_est_meas[6];

        BLA::Matrix<6,1> X;                         // State matrix
        BLA::Matrix<6,6> S;                       // Inverted term in Kalman Gain
        BLA::Matrix<6,6> K;                         // Kalman Gain
        BLA::Matrix<6,1> Y;                         // Measurements

        BLA::Matrix<6,6> A;                   // State mapping
        BLA::Matrix<6,3> B;                   // Command mapping

        BLA::Matrix<6,6, Diagonal<6,float>> P;      // Process noise
        BLA::Matrix<6,6, Diagonal<6,float>> R;      // Measurement Covariance

        BLA::Identity<6,6> H;                       // Identity 

        BLA::Zeros<6,1> W_k;                        // State noise
        BLA::Zeros<6,6> Q_t;                        // Noise in process noise
        BLA::Zeros<6,1> Z_m;                        // Measurement noise

        unsigned long prevTime;
};

#endif