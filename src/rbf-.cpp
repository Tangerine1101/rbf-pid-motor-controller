#include "rbf-.h"
RBFNetwork::RBFNetwork(int num_neurons, const double mu_values[], const double sigma_values[])
    : num_neurons_(num_neurons), mu_internal_(NULL), sigma_internal_(NULL), weights_internal_(NULL) {

    if (num_neurons_ <= 0) {
        // Xử lý lỗi: số nơ-ron không hợp lệ.
        // Trong môi trường Arduino, bạn có thể in ra Serial hoặc đặt một cờ lỗi.
        // Ví dụ: Serial.println("Error: Number of neurons must be positive.");
        num_neurons_ = 0; // Đặt về 0 để các hàm khác biết có lỗi
        return;
    }

    // Cấp phát bộ nhớ cho các mảng nội bộ
    mu_internal_ = new double[num_neurons_];
    sigma_internal_ = new double[num_neurons_];
    weights_internal_ = new double[num_neurons_ * 3]; // 3 cho Kp, Ki, Kd

    if (!mu_internal_ || !sigma_internal_ || !weights_internal_) {
        // Xử lý lỗi cấp phát bộ nhớ
        // Serial.println("Error: Memory allocation failed for RBFNetwork internals.");
        freeMemory(); // Giải phóng bất kỳ phần nào đã được cấp phát
        num_neurons_ = 0; // Đánh dấu là không hợp lệ
        return;
    }

    // Sao chép giá trị mu và sigma
    for (int i = 0; i < num_neurons_; ++i) {
        mu_internal_[i] = mu_values[i];
        sigma_internal_[i] = sigma_values[i];
    }

    // Khởi tạo trọng số ban đầu bằng 0
    for (int i = 0; i < num_neurons_ * 3; ++i) {
        weights_internal_[i] = 0.0;
    }
}

RBFNetwork::~RBFNetwork() {
    freeMemory();
}

void RBFNetwork::freeMemory() {
    delete[] mu_internal_;
    mu_internal_ = NULL;
    delete[] sigma_internal_;
    sigma_internal_ = NULL;
    delete[] weights_internal_;
    weights_internal_ = NULL;
}

bool RBFNetwork::setWeights(const double* weights_data_flat) {
    if (num_neurons_ <= 0 || !weights_internal_) {
        // Serial.println("Error: RBFNetwork not properly initialized to set weights.");
        return false;
    }
    for (int i = 0; i < num_neurons_ * 3; ++i) {
        weights_internal_[i] = weights_data_flat[i];
    }
    return true;
}

bool RBFNetwork::initializePredefinedWeights() {
    if (num_neurons_ != 3 || !weights_internal_) {
        // Serial.println("Error: Predefined weights are for 3 neurons only and network must be initialized.");
        return false;
    }
    // Trọng số được làm phẳng:
    // Neuron 0: Kp, Ki, Kd
    weights_internal_[0] = 2.0;  // N0 Kp
    weights_internal_[1] = 1.5;  // N0 Ki
    weights_internal_[2] = 0.5;  // N0 Kd

    // Neuron 1: Kp, Ki, Kd
    weights_internal_[3] = 1.5;  // N1 Kp
    weights_internal_[4] = 0.9;  // N1 Ki
    weights_internal_[5] = 0.4;  // N1 Kd

    // Neuron 2: Kp, Ki, Kd
    weights_internal_[6] = 1.0;  // N2 Kp
    weights_internal_[7] = 0.5;  // N2 Ki
    weights_internal_[8] = 0.3;  // N2 Kd
    return true;
}

PIDParams RBFNetwork::computePIDParams(double error_input) const {
    PIDParams params = {0.0, 0.0, 0.0};

    if (num_neurons_ <= 0 || !mu_internal_ || !sigma_internal_ || !weights_internal_) {
        // Serial.println("Error: RBFNetwork not properly initialized for computation.");
        return params; // Trả về các tham số 0
    }

    // Sử dụng mảng động tạm thời trên stack (nếu num_neurons_ nhỏ)
    // Hoặc cấp phát động nếu num_neurons_ có thể lớn (nhưng cần giải phóng sau đó)
    // Với số lượng nơ-ron nhỏ như trong ref.cpp (3 nơ-ron), mảng trên stack là ổn.
    // Nếu num_neurons_ có thể lớn, cần cẩn thận với bộ nhớ stack của Arduino.
    double phi[num_neurons_]; // Chú ý: Đây là Variable Length Array (VLA), một phần mở rộng của C99.
                              // Hầu hết các trình biên dịch Arduino (dựa trên GCC) hỗ trợ VLA.
                              // Nếu không, bạn sẽ cần cấp phát động `new double[num_neurons_]` và `delete[]` nó.

    double totalPhi = 0.0;

    for (int i = 0; i < num_neurons_; ++i) {
        // Các hàm pow() và exp() có sẵn từ math.h (thường được bao gồm bởi Arduino.h)
        phi[i] = exp(-pow(error_input - mu_internal_[i], 2) / (2 * pow(sigma_internal_[i], 2)));
        totalPhi += phi[i];
    }

    if (totalPhi > 1e-9) { // Tránh chia cho 0 hoặc số rất nhỏ
        for (int i = 0; i < num_neurons_; ++i) {
            double normalized_phi = phi[i] / totalPhi;
            params.Kp += normalized_phi * weights_internal_[i * 3 + 0]; // Kp weights for neuron i
            params.Ki += normalized_phi * weights_internal_[i * 3 + 1]; // Ki weights for neuron i
            params.Kd += normalized_phi * weights_internal_[i * 3 + 2]; // Kd weights for neuron i
        }
    } else {
        // Chiến lược dự phòng nếu totalPhi quá nhỏ.
        // Ví dụ: sử dụng trọng số của nơ-ron đầu tiên hoặc bộ PID mặc định an toàn.
        // Trong phiên bản này, nó sẽ trả về params với Kp, Ki, Kd = 0.0
        // Serial.println("Warning: totalPhi is near zero. PID params might be zero.");
    }
    return params;
}