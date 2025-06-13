#include <Arduino.h>
#include "rbf-.h"
double normalize(double x, double mean, double std) {
  return (x - mean) / std;
}
double denormalize(double x_norm, double mean, double std) {
  return x_norm * std + mean;
}


// Tính toán giá trị PID từ mạng RBF
void rbf::computePIDRBF(double e_raw) {
  double e = normalize(e_raw, mean_e, std_e);
  double phi[NUM_NEURONS];
  double totalPhi = 0; // Tổng giá trị phi để chuẩn hóa

  for (int i = 0; i < NUM_NEURONS; i++) {
    phi[i] = exp(-pow(e - mu[i], 2) / (2 * pow(sigma[i], 2)));  // Hàm Gaussian
    totalPhi += phi[i];  // Cộng dồn để chuẩn hóa
    //Serial.print("phi["); Serial.print(i); Serial.print("]: "); Serial.println(phi[i]);  // In phi
  }
   //Serial.print("totalPhi: "); Serial.println(totalPhi);  // In totalPhi
  // Tính giá trị Kp, Ki, Kd từ mạng RBF, sử dụng tổng phi để chuẩn hóa
  rbf_Kp = 0;
  rbf_Ki = 0;
  rbf_Kd = 0;

  if (totalPhi > 0) { // Tránh chia cho 0
    for (int i = 0; i < NUM_NEURONS; i++) {
      rbf_Kp += (phi[i] / totalPhi) * weights[i][0];  // Trọng số Kp
      rbf_Ki += (phi[i] / totalPhi) * weights[i][1];  // Trọng số Ki
      rbf_Kd += (phi[i] / totalPhi) * weights[i][2];  // Trọng số Kd
    }
  }
    // Phi chuẩn hóa PID
  rbf_Kp = denormalize(rbf_Kp, mean_Kp, std_Kp);
  rbf_Ki = denormalize(rbf_Ki, mean_Ki, std_Ki);
  rbf_Kd = denormalize(rbf_Kd, mean_Kd, std_Kd);
}
double rbf::getKp(){
  return rbf_Kp;
}
double rbf::getKi(){
  return rbf_Ki;
}
double rbf::getKd(){
  return rbf_Kd;
}
