#ifndef sensormodule_h
#define sensormodule_h

/*
Overview
-------------------------------------------------------------------------------------------------------------
The setup is configured to have 8 ultrasonic sensors that are lined up one side of the plank
with a equal seperation distance.
They are interfaced using an 8 bit Shift-register (SR1).
The 8 output ports from the Shift-register are connected to the Trigger pins of the 8 ultrasonic sensor.
The echo output pins of the Ultrasonic sensors are connected to a 8-bit MUX with 3 control pins.
The three control pins of the MUX are connected to the first three output ports of another 8-bit Shift-Register (SR2)
Input, Serial clock and Register output clock pins of SR1 and SR2 are connected to DIO ports of Arduino.
Additionally, a Laser sensor to detect balls bigger than 1cm is connected to pin3 and a sensor attached to the catcher is 
connected to Pin2 of Arduino.
------------------------------------------------------------------------------------------------------------------

Working

Only one Ultrasonic sensor (US) is active at any given time.
The choice of sensor is controlled by the Shift-register SR1.
Initially, the output register of SR1 is configured to read (00000001), such that only US1 is active.
If an output from the active US is detected, a shift operation on the input to SR1 is performed triggering the next US.
The input to MUX is controlled by SR2 whose input is designed such that echo output from the active US is directed to MUX output.

The Register_output_clock pin is given a pulse with width > 10us to trigger the corresponding the US.
The output_enable pin of the MUX and Shift-Registers (which are Active-low) are connected to ground.
The output pin of MUX is configured to a Pin-Change interrupt.
Therefore, if the echo output of the active US goes high, a trigger is generated.
Similarly, when the echo output of the active US goes low, a trigger is generated.
Timing the length of this output, we can calculate the distance of the marble from the US.
*/



//#define PIN_LS 2
#define PIN_catcher_sensor_output 3
#define PIN_cather_sensor_input 15
#define PIN_SR1_IN 11
#define PIN_SR1_clck 12
#define PIN_SR1_Rout_clck 13
#define PIN_SR2_IN 17   
#define PIN_SR2_clck 18
#define PIN_SR2_Rout_clck 19
#define PIN_MUX_output 16
#define Max_distance_cm_US 50 //Maximum distance to check for response in 
#define US_polling_interval_ms 6 //With Timer2 the maximum interval period is 16ms. For better resoution, use Timer1
// Timer2 prescaler is set at 1024, which is its maximum.
#define US_echo_time_roundtrip_cm 57
#define NoECHO 0

// Flag definitions that detect the status of the ball at various stages

volatile bool marble_detected=0;
volatile bool Trigger_US=0;
volatile bool MUX_output_detected=0;
volatile bool LS_flag=0;
volatile bool catcher_success=0;

const uint8_t seperation_distance_US_cm=12;
uint8_t current_position_marble;
uint8_t horiz_vel_marble,vert_vel_marble; // approximate values for the horizontal and vertical velocities of the marble
uint8_t marble_direction_motion=0; //0 for Left-to-right; 1 for right-to-Left ; 
uint8_t current_US_number=0;
uint8_t SR1_serial_input=1;
uint8_t time_interval_us

void PIN_setup();
void PCINT_attach_interrupt(byte pin);
void PCINT_dettach_interrupt(byte pin);
void setup_timer_interrupt(uint8_t period_ms);

void Trigger_input_SR();
void MUX_control_input_SR();
unsigned int echo_timer_us(); // Function to determine the length of the echo output.
void increment_US();

void determine_motion_direction();
void determine_marble_velocity();
void estimate_final_position();
void measure_distance_cm_US(uint8_t time);






#endif
