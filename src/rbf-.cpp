#include <Arduino.h>
#include "rbf-.h"
rbf::rbf(){
  // Trọng số từ neuron 1 (khi sai số gần 0)
  weights[0][0] = 2.0f;  // Kp
  weights[0][1] = 1.5f;  // Ki
  weights[0][2] = 0.5f;  // Kd

  // Trọng số từ neuron 2 (khi sai số khoảng 20)
  weights[1][0] = 1.5f;  // Kp
  weights[1][1] = 0.9f;  // Ki
  weights[1][2] = 0.4f;  // Kd

  // Trọng số từ neuron 3 (khi sai số lớn, khoảng 100)
  weights[2][0] = 1.0f;  // Kp
  weights[2][1] = 0.5f;  // Ki
  weights[2][2] = 0.3f;  // Kd
}

// Tính toán giá trị PID từ mạng RBF
void rbf::computePIDRBF(double e) {
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