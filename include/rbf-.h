#ifndef RBF_H
#define RBF_H
class rbf {
public:
    rbf(){
  // Weights for Neuron 0
  weights[0][0] = -19.0386;  // Kp
  weights[0][1] = -11.2987;  // Ki
  weights[0][2] = -16.7607;  // Kd

  // Weights for Neuron 1
  weights[1][0] = 7.0994;
  weights[1][1] = 6.4647;
  weights[1][2] = 4.3393;

  // Weights for Neuron 2
  weights[2][0] = 20.0581;
  weights[2][1] = 11.0175;
  weights[2][2] = 19.5430;

  // Weights for Neuron 3
  weights[3][0] = -10.1437;
  weights[3][1] = -8.7381;
  weights[3][2] = -7.5309;

  // Weights for Neuron 4
  weights[4][0] = -24.0053;
  weights[4][1] = -11.9641;
  weights[4][2] = -22.8079;

  // Weights for Neuron 5
  weights[5][0] = -16.9059;
  weights[5][1] = -11.0580;
  weights[5][2] = -16.3760;

  // Weights for Neuron 6
  weights[6][0] = 8.4905;
  weights[6][1] = 5.5721;
  weights[6][2] = 7.9325;

  // Weights for Neuron 7
  weights[7][0] = -2.9149;
  weights[7][1] = -2.6393;
  weights[7][2] = -1.9938;

  // Weights for Neuron 8
  weights[8][0] = 25.3759;
  weights[8][1] = 13.2862;
  weights[8][2] = 23.4092;

  // Weights for Neuron 9
  weights[9][0] = 12.9242;
  weights[9][1] = 9.9586;
  weights[9][2] = 11.2724;
}
    static const int NUM_NEURONS = 10;
    void computePIDRBF(double e);
    double getKp();
    double getKi();
    double getKd();
private:
    // Updated neuron centers
    double mu[NUM_NEURONS] = {1.2192, -1.2008, 0.1894, -0.8404, 0.5499, -0.1452, 1.5788, -1.5589, 0.8845, -0.4799};
    // Updated neuron spread
    double sigma[NUM_NEURONS] = {0.46, 0.46, 0.46, 0.46, 0.46, 0.46, 0.46, 0.46, 0.46, 0.46};
    // Weights matrix to be initialized in the constructor
    double weights[rbf::NUM_NEURONS][3];
    double rbf_Kp, rbf_Ki, rbf_Kd;

    // Updated statistical parameters
    double mean_Kp = 10.7656, std_Kp = 3.6439;
    double mean_Ki = 1.4464, std_Ki = 0.7777;
    double mean_Kd = 27.1718, std_Kd = 13.3598;
    double mean_e = 333.2836, std_e = 77.6861;
};
#endif