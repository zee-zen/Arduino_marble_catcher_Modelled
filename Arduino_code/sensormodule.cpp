#include "sensormodule.h"


void PIN_setup()
{
    //This routine should be called from setup() function on main to initialize all the corresponding ports.
    //The corresponding port numbers should be updated on sensormodule.h file.
    pinMode(PIN_SR1_IN,OUTPUT);
    pinMode(PIN_SR1_clck,OUTPUT);
    pinMode(PIN_SR1_Rout_clck,OUTPUT);
    pinMode(PIN_SR2_IN,OUTPUT);
    pinMode(PIN_SR2_clck,OUTPUT);
    pinMode(PIN_SR2_Rout_clck,OUTPUT);
    pinMode(PIN_MUX_output,INPUT);
    pinMode(PIN_cather_sensor_input,OUTPUT);
    pinMode(PIN_catcher_sensor_output,INPUT);
}

// Attach Pin-Change interrupt on pin (input)
void PCINT_attach_interrupt(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); //enable pin in corresponding PCMSKx (where x is 0, 1 or 2)
    PCIFR |= bit(digitalPinToPCICRbit(pin)); // Clear Flag register of any outstanding interrupts
    PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt of corresponding PCINT on the control register

    // Code adapted from https://playground.arduino.cc/Main/PinChangeInterrupt/.
}

// Dettach Pin-change interrupt on Pin (input)
void PCINT_dettach_interrupt(byte pin)
{
    *digitalPinToPCMSK(pin) = 0;
    PCIFR &= ~(bit(digitalPinToPCICRbit(pin)));
    PCICR &= ~(bit(digitalPinToPCICRbit(pin)));
}

void Timer2_interrupt_setup(uint8_t period_ms)
{
    TCCR2A = 0; //Clearing TC-Control-Register-2-A
    TCCR2B = 0; //Clearing TC-Control-Register-2-B
    TCNT2 = 0; //Clearing Timer2 counter
    OCR2A = ((1000*period_ms)>>6); // Setting value for Output-Compare-Timer2-A. Assuming a prescaler of 1024 and an internal clock freq of 16MHz
    TCCR2A |= (1<<WGM21);
    TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);
    TIMSK2 |= (1<<OCIE2A);

    // Use TIMER2_COMPA_vect for ISR
    // Code adapted from https://www.instructables.com/id/Arduino-Timer-Interrupts/
}
