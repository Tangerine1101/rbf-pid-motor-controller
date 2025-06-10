#include <pid-controller.h>
#include <Arduino.h>

pid::pid(double _Setpoint, double _Kp, double _Ki, double _Kd, double* _input, double* _output){
    kp = _Kp;
    ki = _Ki;
    kd = _Kd;
    setpoint = _Setpoint;
    this->input = _input;
    this->output = _output;
}

void pid::pidCompute(double* control_val){
    error = setpoint - *input;
    *control_val = kp*error + ki*sumE + kd*dE;
    if (*control_val < 0) {
        *control_val = 0;
    }
    else if(*control_val > 1000){
        *control_val = 1000;
    }
    sumE += error*dt;
    dE = (error- lastE )/dt;
    lastE = error;
}

void pid::setPID(double _Setpoint, double _Kp, double _Ki, double _Kd){
    kp = _Kp;
    ki = _Ki;
    kd = _Kd;
    setpoint = _Setpoint;
    lastE =0; dE =0; sumE =0;    
}

void pid::tuneInit(double relayAmp){
    relayH = relayAmp;
    tuneState = 1;
    lastInput = 0;
    peakTime = 0;
    peakMax = setpoint;
    peakMin = setpoint;
}
bool pid::tuning(){
    unsigned long now = millis();
    if (*input < setpoint) {*output = relayH;}
    else {*output = 0;}
    //save overshoot and undershoot
    if(*input > peakMax) peakMax = *input;
    if(*input < peakMin) peakMin = *input;
    //detect if output change direction
    bool crossed_up = (lastInput < setpoint && *input >= setpoint);
    bool crossed_down = (lastInput > setpoint && *input <= setpoint);
    
    if (crossed_up || crossed_down){
        //compute Ziegler-Nichols if there is a peak last time
        if (peakTime != 0) {
            Tu = (double)(now - peakTime)/1000.00;
            double diffPeak = peakMax - peakMin;
            if (diffPeak > 1e-4){
                Ku = (4.0*relayH)/(3.14159*diffPeak);
            }
            //compute Kpid
            double _Kp = 0.6*Ku;
            double _Ki = (1.2*Ku)/Tu;
            double _Kd = (0.6*Ku*Tu)/8.0;
            setPID(setpoint,_Kp,_Ki,_Kd);
            //reset and announce tuning success
            tuneState = 0;
            *output = 0;
            return true;
        }
        //in case there is no peak last time, save time for another peak
        peakTime = now;
        peakMax = setpoint;
        peakMin = setpoint;
    }
    //save input and announce false
    lastInput = *input;
    return false;
}
void pid::tuneCancel(){
    tuneState = 0;
    *output = 0;
    peakTime =0; peakMax =0; peakMin =0;
}
void pid::getPID(double* _Kp, double* _Ki, double* _Kd){
    *_Kp = kp;
    *_Ki = ki;
    *_Kd = kd;
}

//motor related functions
void motor::init(){
  pinMode(pinA, 1);
  pinMode(pinB, 1);
  pinMode(PWM, 1);
  pinMode(enA, 0);
  pinMode(enB, 0);
}
void motor::setPwmFrequency(double frequency) {
  topValue = (int)((16000000.0 / frequency) - 1.0);
  
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
  
  ICR1 = topValue; 
  
  TCCR1B |= (1 << CS10);
}
motor::motor(int _pinA, int _pinB, int _PWM, int _enA, int _enB, volatile long* _pulse){
  pinA = _pinA;
  pinB = _pinB;
  PWM = _PWM;
  enA = _enA;
  enB = _enB;
  this->pulse = _pulse; 
}
void motor::control(int dir, int power) {
  int u = map(power, 0, 1000, 0, topValue);
  if (dir == 0) {
    digitalWrite(pinA, 0);
    digitalWrite(pinB, 0);
    analogWrite(PWM, 0);
  } else if (dir > 0) {
    digitalWrite(pinA, 1);
    digitalWrite(pinB, 0);
    analogWrite(PWM, u);
  } else {
    digitalWrite(pinA, 0);
    digitalWrite(pinB, 1);
    analogWrite(PWM, u);
  }
}
double motor::rpm(){
  unsigned long now = millis();
  noInterrupts();
  long getEnc = *pulse;
  interrupts();
  double dt = (double)(now - lastMeasure)*0.001;
  long pulseDiff = getEnc - lastEnc;
  double u = ((double)pulseDiff*60)/(4*11*9.6*dt);
  switch (rpm_mode)
  {
  case 1:
    filter_medium(&u);
    break;
  case 2:
    filter_lowpass(&u);
  default:
    break;
  }
  lastMeasure = now;
  lastEnc = getEnc;
  return u;
}
void motor::filter_medium(double* val){
  buffer[filterCount] = *val;
  double sum =0.0;
  unsigned int count =0;
  //skip 0 value(for initial period)
  for(unsigned int i =0; i < FILTER_MODE; i++){
    if (buffer[i] >= 0.001) {
      sum += buffer[i];
      count++;
    }
    if (i == (FILTER_MODE -1) && count != 0){
     *val = sum/count;
    }
  }
  filterCount = (filterCount +1) % FILTER_MODE; 
}
void motor::filter_lowpass(double* val){
  double signal = *val;
  double alpha =0;
  if (abs(signal-filteredVal) <= FILTER_THRESHOLD){
    alpha = FILTER_ALPHA_SLOW;
  }
  else{
    alpha = FILTER_ALPHA_FAST;
  }
  signal = (alpha*signal)+((1 - alpha)*filteredVal);
  *val = signal;
}
void motor::config(double frequency, int filterMode){
  rpm_mode = filterMode;
  setPwmFrequency(frequency);
}