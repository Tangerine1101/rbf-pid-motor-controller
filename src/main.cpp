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
bool flag_tuned =0;
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
//get data
const int runLimit = 5;
unsigned int runs_count = 0;
double initialError = Setpoint;
void reset(double sp){
  SysPer_init(&outputVal, &criteria, Setpoint, 2, 25, 0.2, 0.3);
  double Kp, Ki, Kd;
  controller.getPID(&Kp, &Ki, &Kd);
  controller.setPID(sp, Kp, Ki, Kd);
  myMotor.resetCounter();
  wait = millis();
  initialError = Setpoint;
  flag_tuned =0;

}
void resetTuner(double sp, double initE){
  controller.kp = 0;
  controller.ki = 0;
  controller.kd = 0;
  if (sp <= 350){
    controller.tuneInit(1000, initE, pid::modeNoOvershoot);
  }
  else controller.tuneInit(1000, initE, pid::modeLessOvershoot);

}
void printpid(float initError){
  double Kp, Ki, Kd;
  controller.getPID(&Kp, &Ki, &Kd);
  //Serial.print("Setpoint:"); 
  Serial.print(Setpoint); Serial.print(", ");
  //Serial.print("Initial Error:"); 
  Serial.print(initError); Serial.print(", ");
  //Serial.print("Kp:"); 
  Serial.print(Kp); Serial.print(", ");
  //Serial.print("Ki:"); 
  Serial.print(Ki); Serial.print(", ");
  //Serial.print("Kd:"); 
  Serial.println(Kd);
}
void printFail(int count){
    Serial.print("Run fail, time:"); 
    Serial.println(count);
}
//---------------------------------------------------------------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  //CHANGE CRITERIA HERE:
  SysPer_init(&outputVal, &criteria, Setpoint, 2, 25, 0.1, 0.3); //sys_per struct, sys_criteria struct, setpoint, Steady-state error, overshoot(%), time rise(s), time settle(s)
  controller.setPID(Setpoint, 0, 0, 0);
  myMotor.init();
  myMotor.config(15e3, LOW_PASS); //config PWM frequency(not developed yet) and rpm filter mode(LOW_PASS and MEDIUM)
  controller.tuneInit(1000, 0, pid::modeNoOvershoot); //config tuning (High value, Low value, mode)
  attachInterrupt(digitalPinToInterrupt(myMotor.enA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(myMotor.enB), encoderISR, CHANGE);
  while(!Serial);  
  myMotor.control(FORWARD, 100);
  noInterrupts();
  Serial.println("<Taking data>");
  delay(2000);
  interrupts();
  wait = millis();
  }
//---------------------------------------------------------------------------------------------------------------------------------
void loop() {
  for(Setpoint =300; Setpoint <= 500; Setpoint +=10) {
    for(float iniE = 0; iniE <= 1.4; iniE += 0.2) {
        for(int i =0; i <= 5; i++){
            myMotor.control(FORWARD, 0);
            delay(500);
            reset(Setpoint);
            resetTuner(Setpoint, iniE);
            while (runtime() <= 2000) {
                unsigned long t = millis();
                double r = (double)runtime()/1000;
                signal = myMotor.rpm();

                if(controller.tuned(runtime()*0.001)) {

                    if(flag_tuned == 0){
                    myMotor.resetCounter();
                    controller.control_val = iniE*Setpoint;
                    myMotor.control(FORWARD, controller.control_val);
                    //teleplot(runtime()*0.001, signal);
                    flag_tuned  = 1;
                    unsigned long flag_initE = millis();
                    while(millis() - flag_initE <= 1000){
                        signal = myMotor.rpm();
                        unsigned int timer = millis();
                        //teleplot(controller.control_val*0.1, signal);
                        while(millis()- timer <= 10);
                    }
                    initialError = Setpoint-signal;
                    wait = millis();
                    }
                    controller.pidCompute();
                    evaluate(&outputVal, criteria, signal, r);
                }
                myMotor.control(FORWARD, controller.control_val);
                //teleplot(controller.control_val*0.1, signal);
                int crit = meetCriteria(outputVal, criteria)*Setpoint;
                //Serial.print(">criteria:"); Serial.println(crit);
                while((millis() - t) <= 1000*SAMPLE_TIME);
            }

            if (meetCriteria(outputVal, criteria) ){
                printpid(initialError);
                break;
            }
            else{
                //print_performance(outputVal, criteria, 5000);
                //print_criteria(criteria);
                //printFail(i);
            }
        }
    }
  }
}
