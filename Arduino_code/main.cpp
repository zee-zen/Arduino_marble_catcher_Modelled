#include <Arduino.h>
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
5. Experiment is conducted under NTP conditions. Distance measurements using the Ultrasonic sensor 
are dependent on the velocity of sound in air and therefore susceptible to temperature and pressure variations.

Naming conventions:
Ultrasonic sensor : US
Servo motor : motor
Laser sensor - LS

State machine definition

State 1 - Idle state
    Timer interrupt every 20ms
State 2 - Pre-check state
State 3 - Motor commit
State 4 - Motor correction

Psuedo-code

void US1_ISR()
    checkpoint1=True
    number_of_balls+=1

void LS_ISR()
    marble_noncompliant=True
    number_of_balls-=1

void US2_ISR()
    checkpoint2=True

void US3_ISR()
    checkpoint3=True



*/

// ISR to detect output echo pulse from ultrasonic sensors.
// MUX output as defined in sensormodule.h is on port 16 of Arduino. Therefore, the ISR uses the PCINT1_vect interrupt vector.
// If MUX output is connected to D0 to D7 (replace with PCINT2_vect) or D8 - D13 (replace with PCINT0_vect).
ISR(PCINT1_vect)
{
    MUX_output_detected = !(MUX_output_detected);
}


void setup()
{

}

void loop()
{
}
