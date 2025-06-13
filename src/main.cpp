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
double Setpoint = 500;
double signal = myMotor.rpm();
pid controller(Setpoint, &signal);
sys_per outputVal;
sys_criteria criteria;
//operate related
bool flag_run_end =0;
bool flag_tuned =0;
unsigned long wait =0, time_limit = 5000;
unsigned long runtime();
void reset_runtime();
double filteredPT =0;
unsigned int PTmeter();
//rbf
rbf net;

//raw rpm
unsigned long raw_now, raw_lastMeasure;
double raw_lastVal;
long raw_lastEnc;
double rawRPM();

//get data
void getData();
void singleRun();
const int runLimit = 5;
unsigned int runs_count = 0;
double initialError = Setpoint;
void reset(double sp);
void resetTuner(double sp, double initE);
void printpid(float initError);
void printFail(int count);
//---------------------------------------------------------------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  //CHANGE CRITERIA HERE:
  SysPer_init(&outputVal, &criteria, Setpoint, 2, 25, 0.1, 0.3); //sys_per struct, sys_criteria struct, setpoint, Steady-state error, overshoot(%), time rise(s), time settle(s)
  controller.setPID(Setpoint, 0, 0, 0);
  myMotor.init();
  myMotor.config(15e3, LOW_PASS); //config PWM frequency(not developed yet) and rpm filter mode(LOW_PASS and MEDIUM)
  controller.tuneInit(1000, 0, pid::modeHighRespond); //config tuning (High value, Low value, mode)
  attachInterrupt(digitalPinToInterrupt(myMotor.enA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(myMotor.enB), encoderISR, CHANGE);
  while(!Serial);  
  Serial.println("<Taking data v0.2>");
  Serial.println("Set point, initial error, Kp, Ki, Kd, steady state error, steady state value, overshoot, time rise, time settle");
  //myMotor.control(FORWARD, 1000);
  //delay(9000);
  //myMotor.control(STOP, 0);
  //delay(1000);
  enCount =0;
  reset_runtime();
  }
//---------------------------------------------------------------------------------------------------------------------------------
void loop() {
  singleRun();
}
void rbf_pid(double _setpoint, double _error){

}
unsigned int PTmeter(){
  int Ao = analogRead(A0);
  double u = (0.1*Ao)+(0.9*filteredPT);
  filteredPT = u;
  return filteredPT;
}
void getData1(){
  for(Setpoint =200; Setpoint <= 500; Setpoint +=4) {
    for(float iniE = 0; iniE <= 1.0; iniE += 0.1) {
        for(int i =0; i < 3; i++){
            myMotor.control(FORWARD, 0);
            delay(100);
            reset(Setpoint);
            resetTuner(Setpoint, iniE);
            while (runtime() <= 1500) {
                unsigned long t = millis();
                long r = (long)runtime();
                signal = myMotor.rpm();

                if(controller.tuned(runtime()*0.001)) {

                    if(flag_tuned == 0){
                    myMotor.resetCounter();
                    controller.control_val = 1.5*iniE*Setpoint;
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
                    reset_runtime();
                    }
                    controller.pidCompute();
                    evaluate(&outputVal, criteria, signal, r);
                }
                myMotor.control(FORWARD, controller.control_val);
                //teleplot(controller.control_val*0.1, signal);
                //int crit = meetCriteria(outputVal, criteria)*Setpoint;
                //Serial.print(">criteria:"); Serial.println(crit);
                while((millis() - t) <= 1000*SAMPLE_TIME);
            }

            if (meetCriteria(outputVal, criteria) ){
                printpid(initialError);
                print_performance(outputVal, criteria, 1);
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
  Serial.println("<finish v0.2>");
  while(true);
}

void singleRun(){
  while (runtime() <= 5000 ) {
    unsigned long t = millis();
    long r = (long)runtime();
    signal = myMotor.rpm();

    if(controller.tuned(runtime()*0.001)) {
        if(flag_tuned == 0){
        myMotor.resetCounter();
        //teleplot(runtime()*0.001, signal);
        flag_tuned  = 1;
        initialError = Setpoint-signal;
        delay(1000);
        reset_runtime();
        }
        controller.pidCompute();
        bool i =evaluate(&outputVal, criteria, signal, r);
    }
    myMotor.control(FORWARD, controller.control_val);
    teleplot(controller.control_val*0.1, signal);
    int crit = meetCriteria(outputVal, criteria)*Setpoint;
    Serial.print(">criteria:"); Serial.println(crit);
    Serial.print(">T:"); Serial.println(runtime()*0.001);
    while((millis() - t) <= 1000*SAMPLE_TIME);
}

if (meetCriteria(outputVal, criteria) ){
    printpid(initialError);
    print_performance(outputVal, criteria, 1);
}
else{

    print_performance(outputVal, criteria, 5000);
    print_criteria(criteria);
    printFail(0);
}
myMotor.control(STOP, 0);
while(true);
}
void reset(double sp){
  enCount =0;
  SysPer_init(&outputVal, &criteria, Setpoint, 2, 25, 0.2, 0.3);
  double Kp, Ki, Kd;
  controller.getPID(&Kp, &Ki, &Kd);
  controller.setPID(sp, Kp, Ki, Kd);
  myMotor.resetCounter();
  reset_runtime();
  initialError = Setpoint;
  flag_tuned =0;

}
void resetTuner(double sp, double initE){
  enCount =0;
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
  Serial.print(Kd); Serial.print(", ");
}
void printFail(int count){
    Serial.print("Run fail, time:"); 
    Serial.println(count);
}
unsigned long runtime() {
  if ( ((float)millis() - (float)wait) >= 0){
    return (millis() - wait);
  }
  else {
    return 0;
  }
}
void reset_runtime(){
  wait = millis();
  outputVal.timer = 0;
}
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