#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>
#ifndef SAMPLE_TIME
#define SAMPLE_TIME 0.01
#endif
class pid {
    public:
    //Constructor
        pid(double _Setpoint, double _Kp, double _Ki, double _Kd, double* _input);
    //controller
        double setpoint, control_val;
    //function
        //controller
        void pidCompute();
        void setPID(double _Setpoint, double _Kp, double _Ki, double _Kd);
        //tuner
        void tuneInit(double relayAmp);
        bool tuning();
        void tuneCancel();
        void getPID(double* _Kp, double* _Ki, double* _Kd);
        void getTune();
        private:
    //controller
        double* input;
        double kp, ki, kd;
        double error;
        double lastE =0, dE =0, sumE =0;
        const double dt = SAMPLE_TIME; 
    //tuner
        unsigned long now =0;
        bool tuneState =0;
        double peakMax =1e-9;
        double peakMin =1e-9;
        double relayH = 255; //control value at high
        double lastInput =1e-9;
        double peakTime = 1e-9;
        double Tu = 1e-9;
        double Ku = 1e-9;
};

//motor control
class motor {
  public:
    motor(int _pinA, int _pinB, int _PWM, int _enA, int _enB, volatile long* _pulse);
    #define FORWARD 1
    #define STOP 0
    #define REVERSE -1
    //CONFIG HERE:
    #define FILTER_MODE 3
    #define FILTER_ALPHA_FAST 0.7 //the lower alpha is the delayer the signal is
    #define FILTER_ALPHA_SLOW 0.1 
    #define FILTER_THRESHOLD 0.1 //percentage
    //measure related
    volatile long* pulse;
    long lastEnc = 0;
    unsigned long lastMeasure =0;
    double filteredVal =0;
    double rpm();
    void filter_medium(double* val);
    void filter_lowpass(double* val);
    //control related
    int pinA, pinB, PWM, enA, enB;    
    void control(int dir, int power);
    //config
    #define NO_FILTER 0
    #define MEDIUM 1
    #define LOW_PASS 2
    void setPwmFrequency(double frequency);
    void config(double frequency, int filterMode);
    void init();
  private:
    int rpm_mode =0;
    int topValue =255;
    double buffer[FILTER_MODE];
    int filterCount =0;
};

#endif