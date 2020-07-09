#include<Arduino.h>
#include "NewPing.h"
#include "sensormodule.h"
#include "motorcontrol.h"
#include "PID_v1.h"

/*
Strategy and design.
Board dimensions - 1m x 0.5m
Incline angle of the board - unknown
Sensors and actuators used:
1. 5 ultrasonic distance sensors to measure distance
2. Laser transmitter and receiver module to enforce size requirement of marble
3. Servo motor to control the position of the catcher bin.

Assumptions and boundary conditions
1. Marble is released only from the top.
2. Marble rolls down the plank and does not bounce around.
3. Plank is not too rough or has imperfections that affect the trajectory of the marble significantly
4. Setup is located in an environment with ambient high frequency noise. (roughly 40KHz in our case)

Naming conventions:
Ultrasonic sensor : US
Servo motor : motor
Laser sensor - LS

State machine definition

State1 (Idle state): No marble detected on the board
Timer interrupt every 10ms to ping US1, which is located at the very top. US1 would be the sensor that first
detects the presence of any object.
If no object is detected, then US1 returns a Low (0).
If an object is detected, then US1 returns the distance of the object from it.
current time = millis()
State transition to State2

State2 (Pre-check state): Timer interrupt for every 10ms to ping US1 continues.
Setup LS to pin-change interrupt.




*/


void setup()
{

}

void loop()
{
}
