#include "WProgram.h"
#include "MotorControl.h"

MotorControl::MotorControl(int pinFwd,int pinRev, int pinPwm)
{
 pinMode(pinFwd, OUTPUT);
 pinMode(pinRev, OUTPUT);
 _pinFwd = pinFwd;
 _pinRev = pinRev;
 _pinPwm = pinPwm;
}

void MotorControl::SpeedWrite(int d,int Pwm)  /* Pass values as SpeedWrite(1/0, value by pid)*/
{
 _Pwm = Pwm;
 _d=d; /* This value is 1 for forward rotation and 0 for reverse rotation*/

 // Check if PWM value do not exceed the boundaries (0 to 255).
 _Pwm = constrain(_Pwm, 0, 255);

 // Is the PWM value positive, if yes go forward.
 if (_Pwm >= 1 && _Pwm <= 255)
 {if (_d=1)
   {
   digitalWrite(_pinFwd, HIGH );
   digitalWrite(_pinRev, LOW);
   analogWrite (_pinFwd, _Pwm);
   _Direction = 'F';
   }
  else if (d=0)
   {
   digitalWrite(_pinFwd, LOW );
   digitalWrite(_pinRev, HIGH);
   analogWrite (_pinFwd, _Pwm);
   _Direction = 'R';
   }
 }

 // Is the PWM value zero, if yes the motor is idle.
 else if (_Pwm == 0)
 {
   digitalWrite(_pinFwd, HIGH);
   digitalWrite(_pinRev, HIGH);
   analogWrite (_pinFwd, _Pwm);
   _Direction = 'B';
 }
}

void MotorControl::SpeedStepUp(int PwmStepUp)
{
 _PwmStepUp = PwmStepUp;

 _Pwm = _Pwm + _PwmStepUp;

 this->SpeedWrite(_d,_Pwm);
}

void MotorControl::SpeedStepDown(int PwmStepDown)
{
 _PwmStepDown = PwmStepDown;

 _Pwm = _Pwm - _PwmStepDown;

 this->SpeedWrite(_d,_Pwm);
}

int MotorControl::SpeedRead()
{
 return _Pwm;
}

char MotorControl::Direction()
{
 return _Direction;
}
