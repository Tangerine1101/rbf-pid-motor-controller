#ifndef RBFNETWORK_H
#define RBFNETWORK_H

// Arduino.h thường bao gồm các hàm toán học cơ bản như exp, pow
// Nếu không, bạn có thể cần #include <math.h> một cách tường minh
#include <Arduino.h> // Hoặc #include <math.h>

// Cấu trúc để lưu trữ các tham số PID
struct PIDParams {
    double Kp;
    double Ki;
    double Kd;
};

class RBFNetwork {
public:
    // Constructor: khởi tạo với số lượng nơ-ron, con trỏ tới mảng tâm và độ rộng
    // Người dùng chịu trách nhiệm về việc các mảng mu_values và sigma_values tồn tại
    // trong suốt thời gian sống của đối tượng RBFNetwork nếu không copy sâu.
    // Để đơn giản và an toàn hơn, chúng ta sẽ copy các giá trị này vào mảng nội bộ.
    RBFNetwork(int num_neurons, const double mu_values[], const double sigma_values[]);

    // Destructor để giải phóng bộ nhớ đã cấp phát (nếu có)
    ~RBFNetwork();

    // Thiết lập trọng số cho mạng
    // weights_data phải là một mảng 1D được làm phẳng [num_neurons * 3]
    // theo thứ tự: w_n0_kp, w_n0_ki, w_n0_kd, w_n1_kp, w_n1_ki, w_n1_kd, ...
    // Trả về true nếu thành công, false nếu có lỗi (ví dụ: num_neurons không khớp)
    bool setWeights(const double* weights_data_flat);

    // Tính toán các tham số PID dựa trên sai số đầu vào
    PIDParams computePIDParams(double error_input) const;

    // (Tùy chọn) Khởi tạo trọng số với các giá trị cố định như trong file ref.cpp
    // Chỉ hoạt động nếu num_neurons_ được khởi tạo là 3.
    bool initializePredefinedWeights();

private:
    int num_neurons_;
    double* mu_internal_;      // Mảng nội bộ để lưu trữ các giá trị tâm
    double* sigma_internal_;   // Mảng nội bộ để lưu trữ các giá trị độ rộng
    double* weights_internal_; // Mảng nội bộ 1D được làm phẳng cho trọng số (kích thước: num_neurons * 3)

    // Hàm phụ trợ để cấp phát bộ nhớ (nếu bạn muốn tách logic này ra)
    // void allocateMemory();
    void freeMemory();
};

#endif // RBFNETWORK_H