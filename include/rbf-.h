#ifndef RBF_H
#define RBF_H
#include<Arduino.h>
class rbf {
    public:
        rbf();
        static const int NUM_NEURONS = 3;
        void computePIDRBF(double e);
        double getKp();
        double getKi();
        double getKd();
    private:
        double mu[NUM_NEURONS] = {0, 20, 100};  // Trung tâm neuron cho các giá trị sai số
        double sigma[NUM_NEURONS] = {10, 15, 25};  // Độ rộng của hàm Gaussian
        double weights[rbf::NUM_NEURONS][3];
        double rbf_Kp, rbf_Ki, rbf_Kd;

};
#endif