#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>
#ifndef SAMPLE_TIME
#define SAMPLE_TIME 0.01
#endif
class pid {
    public:
    //Constructor
        pid(double _Setpoint, double* _input);
    //controller
        double setpoint, control_val;
    //function
        //controller
        void pidCompute();
        void setPID(double _Setpoint, double _Kp, double _Ki, double _Kd);
        //tuner
        typedef enum {
            modeBasic,
            modeLessOvershoot,
            modeNoOvershoot,
            modeHighRespond,
            modeNoTune
        } ZNmode;
        void tuneInit(double high, double low, ZNmode zn);
        void tuneCancel();
        void getPID(double* _Kp, double* _Ki, double* _Kd);
        void getTune(long time);
        bool tuned(unsigned long run_time);
    //controller
        double* input;
        double kp, ki, kd;
        double error;
    //------------------
        private:
        #define Pi 3.14159265358979323846
        double lastE =0, dE =0, sumE =0;
        //dE filter
        double filteredDe;
        double alpha = 0.1;
        void lowpassFilter(double* val);
        const double dt = SAMPLE_TIME; 
    //tuner
        ZNmode ZN_Mode;
        bool tuneState =0;
        double peakMax =1e-9;
        double peakMin =1e-9;
        double relayON = 255; //control value at high
        double relayOFF = 0;
        int tune_cached;
        double _Tu = 1e-9;
        double _Ku = 1e-9;

        unsigned long t1, t2, tHigh, tLow;
        int ZN_count;
        const int ZN_cycle = 10;

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
    #define FILTER_ALPHA_FAST 0.4 //the lower alpha is the delayer the signal is
    #define FILTER_ALPHA_SLOW 0.1
    #define FILTER_THRESHOLD 0.35 //percentage
    //measure related
    volatile long* pulse;
    long lastEnc = 0;
    unsigned long lastMeasure =0, now =0;
    double filteredVal =0;
    double rpm();
    void resetCounter();
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
    double spd;
    int rpm_mode =0;
    int topValue =255; //max analogWrite output
    double buffer[FILTER_MODE];
    int filterCount =0;
};

#endif