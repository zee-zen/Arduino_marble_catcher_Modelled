#include <Arduino.h>
#include "Math.h"
#include "NewPing.h"
#include "MotorControl.h"
#include "PID_v1.h"
//#include "stuff.h"

#define ADC_ref 5 // ADC reference Voltage
#define zero_x 1.799
#define zero_y 1.799
#define zero_z 1.799
#define sensitivity_x 0.4
#define sensitivity_y 0.4
#define sensitivity_z 0.4
unsigned int value_x;
unsigned int value_y;
unsigned int value_z;
float xv;
float yv;
float zv;
float angle;
const float mu=0.2;
const float g=9800;

//#define pid_input_pin /*denote pin of ultrasonic sensor*/
#define pid_output_pin /*denote pin of H-bridge enable*/
#define TRIGGER_PIN1  12  // Arduino pin tied to trigger pin on ping sensor1.
#define ECHO_PIN1     11  // Arduino pin tied to echo pin on ping sensor1.
#define TRIGGER_PIN2  4  // Arduino pin tied to trigger pin on ping sensor2.
#define ECHO_PIN2     5  // Arduino pin tied to echo pin on ping sensor2.
#define TRIGGER_PIN3  6  // Arduino pin tied to trigger pin on ping sensor3.
#define ECHO_PIN3     7  // Arduino pin tied to echo pin on ping sensor3.
#define TRIGGER_PIN4  8  // Arduino pin tied to trigger pin on ping sensor4.
#define ECHO_PIN4     9  // Arduino pin tied to echo pin on ping sensor4.
#define MAX_DISTANCE 102 // Maximum distance we want to ping for (in centimeters). Rated at 400-500cm.
#define interrupt_pin 10
#define laser_pin 2
#define motor_pin1 1 
#define motor_pin2 0 
#define motor_pin3 3
#define ir_interrupt A3

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);
NewPing sonar4(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE);

unsigned int pingSpeed = 5; // How frequently are we going to send out a ping (in milliseconds).
unsigned long pingTimer_S1;     // Holds the next ping time.
unsigned long pingTimer_S2;
unsigned long pingTimer_S3;
unsigned long pingTimer_S4;
unsigned long start_time;
unsigned long finish_time;

double pid_Setpoint, pid_Input, pid_Output;

double Kp=3, Ki=5, Kd=1;

PID feed_pid(&pid_Input, &pid_Output, &pid_Setpoint, Kp, Ki, Kd, 1);
MotorControl movemotor(motor_pin1,motor_pin2,motor_pin3)
typedef enum {State1,State2,State3,State4,State5}State;
State next_state;

void setup() {
analogReference(ADC_ref);
next_state=State1;
  // put your setup code here, to run once:
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pingTimer_S1 = millis();
  pingTimer_S2 = millis(); // Start now.;
 //turn the PID on
feed_pid.SetMode(1);
//initialise motor
movemotor.SpeedWrite(1,0);
}

void loop() {int distance1,distance2,distance3,distance4,First_val,last_val,difference,move,last_position,extra;
int *ptr1=&move; boolean motor_dir; int interrupt; int pwm_generated;int ir_counter; int interrupt_ir;

value_x = analogRead(A0);
value_y = analogRead(A1);
value_z = analogRead(A2);
xv=(value_x/1024.0*ADC_ref-zero_x)/sensitivity_x;
yv=(value_y/1024.0*ADC_ref-zero_y)/sensitivity_y;
zv=(value_z/1024.0*ADC_ref-zero_z)/sensitivity_z;
angle =atan2(-yv,-zv)*57.2957795+180;
  // put your main code here, to run repeatedly:
  pid_Setpoint = move;
  pid_Input = distance4;
  feed_pid.Compute();
  pwm_generated=analogRead(motor_pin3);
  analogWrite(motor_pin3,pid_Output);
  //analogWrite(pid_output_pin, pid_Output);
  movemotor.SpeedWrite(motor_dir,pwm_generated);  

switch (next_state)
{case State1:
if (millis() >= pingTimer_S1) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer_S1 += pingSpeed;      // Set the next ping time.
    distance1=sonar1.ping_cm();}
    if (distance1=0){next_state=State1;}
    else if (distance1>0){
      start_time=millis();
      First_val=distance1;
      digitalWrite(laser_pin,HIGH);
      next_state=State2;}
      break;

case State2: 
  interrupt=digitalRead(interrupt_pin);
  if (interrupt=1)
  {if (millis() >= pingTimer_S2) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer_S2 =millis()+1  ;      // Set the next ping time in ms.
    distance2=sonar2.ping_cm();}
    if (distance2=0){next_state=State2;}
    else if (distance2>0){
      finish_time=millis();
      last_val=distance2; 
      digitalWrite(laser_pin,LOW);
      next_state=State3;
    }
  }
  else if (interrupt=0)
    {next_state=State1;}
    break;
  
Case State3:
  int difference=distance2-distance1;
  int total_time=finish_time-start_time;
  int v_x=4.5/total_time;
  int v_y=difference/total_time;
  int v_xsq=sq(v_x);
  int v_ysq=sq(v_y);
  float s_theta=v_y/(sqrt(v_xsq+v_ysq));
  int predicted_y=((v_y*v_x)-0.5*v_xsq)/(mu*g*cos(angle)*s_theta);
  if (difference>0)
  {int last_position=distance1+predicted_y;
    if (last_position>50)
    {motor_dir=1;
    move=abs(last_position-move);}
    else if (last_position<50)
    {motor_dir=0;
    move=abs(last_position-move);}
  }
  else if (difference<0)
  {int last_position=distance1-predicted_y;
    if (last_position>50)
      {motor_dir=1;
      move=abs(last_position-move);}
    else if (last_position<50)
      {motor_dir=0;
      move=abs(last_position-move);
      }  
  }
  if (last_position<0 or last_position>100){next_state=State1;}
  else if (millis() >= pingTimer_S4) {  
    pingTimer_S4 =millis()+pingSpeed  ;  
    distance4=sonar4.ping_cm();
  next_state=State4;}
  break;

Case State4:
if (millis() >= pingTimer_S3) {  
    pingTimer_S3 =millis()+5  ;  
    distance3=sonar3.ping_cm();}
    if (distance3=0){next_state=State4;}
    else if (distance3>0)
    {if (distance3!=last_position)
      {extra=distance3-last_position;
      if (extra>3)
      {*ptr1=*ptr1+extra;}
      else if (extra<3)
      {*ptr1=*ptr1;}
    }
    ir_counter=millis();
    next_state=State5;}
if (millis() >= pingTimer_S4) {  
  pingTimer_S4 =millis()+pingSpeed  ;  
  distance4=sonar4.ping_cm();}
    break;

Case State5:
interrupt_ir=digitalRead(ir_interrupt);
if (interrupt_ir=0)
    {move=50;
    motor_dir=not motor_dir;
    next_state=State1;}
  else if (interrupt_ir=1)
    {next_state=State5;
    if (millis()>ir_counter+pingSpeed)
    {next_state=State1;}
    }
if (millis() >= pingTimer_S4) {  
  pingTimer_S4 =millis()+pingSpeed  ;  
  distance4=sonar4.ping_cm();}
break;
}
  
    
