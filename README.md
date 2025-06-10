# rbf-pid-motor-controller
I. Idea
This is a rbf-pid controller to control motor's speed by use a RBF network to tuning pid controller real-time base on set point and error.
II. Module
pid-controller - contain pid and motor controller, also calculate speed and refine it.
system performance - to evaluate the signal and handling data
rbf - the RBF neuron network and training function, under developing
III. process 
1. get data
Get data by using tuning algorithm to get pid parameter then evaluate if it satisfy criteria or not.
Step:
- Give the program a vector of set point and error at start = set point.
- Tune and run with the set point vector
- get data and train the rbf network
note: base on the motor you use, you might want to config the filter(even not use filter)
2. online training
With the data above, the rbf network now is ready to training online:
- put the motor to a fixed speed(using pid controller) that already have pid's parameters at get data step.
- use rbf-pid to control motor with slightly higher the current speed
3. deploy