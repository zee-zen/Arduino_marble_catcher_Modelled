#include<Arduino.h>
#include "sensormodule.h"
#include "MotorControl.h"
#include "PID_v1.h"

#define pid_input_pin /*denote pin of ultrasonic sensor*/
#define pid_output_pin /*denote pin of H-bridge enable*/
long pid_Setpoint, pid_Input, pid_Output;

byte Kp=3, Ki=5, Kd=1;

PID feed_pid(&pid_Input, &pid_Output, &pid_Setpoint, Kp, Ki, Kd, 1);
MotorControl movemotor(pin1,pin2,pin3)

void setup()
{
pid_Input = analogRead(pid_input_pin);
 //turn the PID on
feed_pid.SetMode(1);
//initialise motor
movemotor.SpeedWrite(1,0);
}

void loop()
{
  /*  pid_Setpoint = analogRead(Mux output);
  pid_Input = analogRead(pid_input_pin);
  feed_pid.Compute();
  analogWrite(pid_output_pin, pid_Output);
  movemotor.SpeedWrite(1/0,pid_ouput);
  
*/
}
