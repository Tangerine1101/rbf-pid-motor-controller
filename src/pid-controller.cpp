#include <pid-controller.h>
#include <Arduino.h>


pid::pid(double _Setpoint, double* _input){
    setpoint = _Setpoint;
    this->input = _input;
}

void pid::pidCompute(){
    error = setpoint - *input;
    sumE += error;
    dE = (error- lastE );
    if (control_val < 1000 && control_val >0)
    sumE = constrain(sumE, 0, 1000);
    lowpassFilter(&dE);
    double pid_val = kp*error + ki*sumE + kd*dE;
    control_val = constrain(pid_val, 0, 1000);
    lastE = error;
}


void pid::lowpassFilter(double* val){
    double signal = *val;
    filteredDe = signal;
    signal = (alpha*signal)+((1-alpha)*filteredDe);
    *val = signal;
}
void pid::setPID(double _Setpoint, double _Kp, double _Ki, double _Kd){
    kp = _Kp;
    ki = _Ki;
    kd = _Kd;
    setpoint = _Setpoint;
    lastE =0; dE =0; sumE =0;    
    filteredDe = 0;
}

void pid::tuneInit(double high, double low, ZNmode zn){
    relayON = high;
    relayOFF = low;
    ZN_Mode = zn;
    ZN_count = 0;
    peakMax = setpoint;
    peakMin = setpoint;
    t1 = 0;
    t2 = 0;
    if (zn == pid::modeNoTune){
      tuneState = 0;
    }
    else tuneState = 1;
}
bool pid::tuned(unsigned long run_time){
  if(tuneState ==0) return 1;

  double kpConst, tiConst, tdConst;
  if (ZN_Mode == modeBasic){
    kpConst = 0.6;
    tiConst = 0.5;
    tdConst = 0.125;
  }
  else if (ZN_Mode == modeLessOvershoot){ //overshoot is 10-20% base on setpoint and ~0.2s settling time, not recommence for set point < 350 since higher overshoot and longer settling time
    kpConst = 0.15;
    tiConst = 1.2;
    tdConst = 0.33;

  }
  else if (ZN_Mode == modeNoOvershoot){ // overshoot ~5%, won't work on high set point. Best on set point = 200 - 350 
    kpConst = 0.06;
    tiConst = 1.8;
    tdConst = 0.33;
    
  }
  else if (ZN_Mode == modeHighRespond){
    kpConst = 0.7;
    tiConst = 0.5;
    tdConst = 0.125;
    
  }
  else {
    return 1;
  }
  //save overshoot and undershoot
  if(*input > peakMax) peakMax = *input;
  if(*input < peakMin) peakMin = *input;
  //relay is on and input signal reached the setpoint
  if(control_val == relayON && *input >= setpoint){
    control_val = relayOFF;
    t1 = millis();
    tHigh = t1 - t2;
  }
  if(control_val != relayON && *input < setpoint){
    control_val = relayON;
    t2 = millis();
    tLow = t2 - t1;
    double a = (peakMax - peakMin)/2.0;
    double h = relayON - relayOFF;
    
    if (a >= 1e-9) {
      double Ku = (4.0*h)/(Pi*a);
      double Tu = (tLow + tHigh)*0.001;

      double _Kp = kpConst*Ku;
      double _Ki = _Kp/(tiConst*Tu)*SAMPLE_TIME;
      double _Kd = (tdConst*_Kp*Tu)/SAMPLE_TIME;

      if(ZN_count >= 1){
        kp += _Kp;
        ki += _Ki;
        kd += _Kd;
      }
    }
    //reset minimum
    peakMin = setpoint;
    peakMax = setpoint;
    ZN_count ++;
  }
  //if get enough cycle, compute average
  if (ZN_count >= ZN_cycle){
    control_val = relayOFF;
    kp = kp/(ZN_count - 1);
    ki = ki/(ZN_count - 1);
    kd = kd/(ZN_count - 1);
    tuneState = 0;
    getTune(run_time);
    return 1;
  }
  return 0;
}

void pid::getTune(long time){
  double Kp, Ki, Kd;
  getPID(&Kp, &Ki, &Kd);
  Serial.print("Tuned! "); Serial.println(time);
  Serial.print("Kp:"); Serial.println(Kp);
  Serial.print("Ki:"); Serial.println(Ki);
  Serial.print("Kd:"); Serial.println(Kd);
}
void pid::tuneCancel(){
    tuneState = 0;
    control_val = 0;
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
  now = millis();
  if ((now - lastMeasure) <= 1000*SAMPLE_TIME) return filteredVal;
  noInterrupts();
  long getEnc = *pulse;
  interrupts();
  double dt = (double)(now - lastMeasure)*0.001;
  if (dt <= 1e-5) return filteredVal;
  long pulseDiff = getEnc - lastEnc;
  double u =0;
  u = ((double)pulseDiff*60)/(4*11*9.6*dt);
  if (u <= 1){
    u = 0;
    filteredVal = 0;
  }
  switch (rpm_mode)
  {
  case 1:
    filter_medium(&u);
    break;
  case 2:
    filter_lowpass(&u);
    break;
  default:
    break;
  }
  filteredVal = u;
  lastMeasure = now;
  lastEnc = getEnc;
  return u;
}
void motor::resetCounter() {
  *pulse =0;
  filteredVal = 0;
  lastMeasure = 0;
  lastEnc = 0;
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
  if ((abs(signal-filteredVal)/signal) <= FILTER_THRESHOLD){
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