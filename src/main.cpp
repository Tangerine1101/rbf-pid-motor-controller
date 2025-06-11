#ifndef SAMPLE_TIME
#define SAMPLE_TIME 0.01
#endif

#include <Arduino.h>
#include"pid-controller.h"
#include"system_performance.h"
#include"rbf-.h"
//motor & encoder related
volatile long enCount = 0;
void encoderISR(){
  enCount++;
}
motor myMotor(7, 8, 9, 2, 3, &enCount);
//pid & system evaluation related
double Setpoint =200;
double signal = myMotor.rpm();
pid controller(Setpoint, 1, 1, 1, &signal);
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
//---------------------------------------------------------------------------------------------------------------------------------
void setup(){
  Serial.begin(9600);
  //CHANGE CRITERIA HERE:
  SysPer_init(&outputVal, &criteria, Setpoint, 2, 1, 0.1, 0.1); //sys_per struct, sys_criteria struct, setpoint, Steady-state error, overshoot(%), time rise(s), time settle(s)
  myMotor.init();
  myMotor.config(15e3, LOW_PASS);
  attachInterrupt(digitalPinToInterrupt(myMotor.enA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(myMotor.enB), encoderISR, CHANGE);

  while(!Serial);  
  wait = millis();
  }
//---------------------------------------------------------------------------------------------------------------------------------
void loop() {
  unsigned long t = millis();
  unsigned long r = runtime();
    if(controller.tuning() == 1){
      controller.pidCompute();
      evaluate(&outputVal, criteria, myMotor.rpm(), (double)runtime()/1000);
    }
    myMotor.control(FORWARD, controller.control_val);
    teleplot(runtime(), controller.control_val);

    signal = myMotor.rpm();
    while(millis() - t < 1000*SAMPLE_TIME);
  /*if (flag_run_end == 0) {
    print_performance(outputVal, criteria, 5.0);
    flag_run_end =1;
  }*/
}

