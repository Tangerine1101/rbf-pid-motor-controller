#ifndef SAMPLE_TIME
#define SAMPLE_TIME 0.01
#endif

#include <Arduino.h>
#include"pid-controller.h"
#include"system_performance.h"
#include"rbf-.h"
#include<time.h>
//motor & encoder related
volatile long enCount = 0;
void encoderISR(){
  enCount++;
}
motor myMotor(7, 8, 9, 2, 3, &enCount);
//pid & system evaluation related
double Setpoint = 200;
double signal = myMotor.rpm();
pid controller(Setpoint, &signal);
sys_per outputVal;
sys_criteria criteria;
//operate related
bool flag_run_end =0;
unsigned long wait =0, time_limit = 5000;
unsigned long runtime() {
  if ( ((float)millis() - (float)wait) >= 0){
    return (millis() - wait);
  }
  else {
    return 0;
  }
}
//raw rpm
unsigned long raw_now, raw_lastMeasure;
double raw_lastVal;
long raw_lastEnc;
double rawRPM(){
  raw_now= millis();
  if ((raw_now - raw_lastMeasure) <= 1000*SAMPLE_TIME) return raw_lastVal;
;
  noInterrupts();
  long getEnc = enCount;
  interrupts();
  double dt = (double)(raw_now - raw_lastMeasure)*0.001;
  if (dt <= 1e-5) return raw_lastVal;
  long pulseDiff = getEnc - raw_lastEnc;
  double u =0;
  u = ((double)pulseDiff*60)/(4*11*9.6*dt);
  raw_lastVal = u;
  raw_lastMeasure = raw_now;
  raw_lastEnc = getEnc;
  return u;
}
//---------------------------------------------------------------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  //CHANGE CRITERIA HERE:
  SysPer_init(&outputVal, &criteria, Setpoint, 2, 1, 0.1, 0.1); //sys_per struct, sys_criteria struct, setpoint, Steady-state error, overshoot(%), time rise(s), time settle(s)
  controller.setPID(Setpoint, 8.9, 3.2, 6.12);
  myMotor.init();
  myMotor.config(15e3, LOW_PASS); //config PWM frequency(not developed yet) and rpm filter mode(LOW_PASS and MEDIUM)
  controller.tuneInit(1000, 0, pid::modeNoOvershoot); //config tuning (High value, Low value, mode)
  attachInterrupt(digitalPinToInterrupt(myMotor.enA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(myMotor.enB), encoderISR, CHANGE);
  while(!Serial);  
  myMotor.control(FORWARD, 100);
  noInterrupts();
  delay(100);
  interrupts();
  wait = millis();
  }
//---------------------------------------------------------------------------------------------------------------------------------
void loop() {
  bool flag_tuned =0;
  while (runtime() <= 5000) {
  unsigned long t = millis();
  double r = (double)runtime()/1000;
  signal = myMotor.rpm();

  if(controller.tuned(runtime())) {
    if(!flag_tuned){
      myMotor.control(STOP, 0);
      myMotor.resetCounter();
      signal = myMotor.rpm();
      teleplot(runtime()*0.001, signal);
      delay(1000);
      wait = millis();
      flag_tuned  = 1;
    }
    controller.pidCompute();
    evaluate(&outputVal, criteria, signal, r);
    printCSV(runtime()*0.001, controller.control_val);
    Serial.print(">raw_speed:"); Serial.println(rawRPM());
  }
  myMotor.control(FORWARD, controller.control_val);
  teleplot(runtime()*0.001, signal);

  while((millis() - t) <= 1000*SAMPLE_TIME);
  }

  if(!flag_run_end) {
    myMotor.control(STOP, 0);
    flag_run_end = 1;
    print_performance(outputVal, criteria, 5000);
  }
}

