#include "dcpwm.h"

/* ESP32 has 2 Motorcontroller PWM units with 3 timers, 3 operators and 6 PWM outputs
*/

bool DCPWM::init(int frequency, int resolution, int channelA,int channelB,  int pinA, int pinB)
{
  _frequency = frequency;
  _resolution = resolution;
  _channelA = channelA;
  _channelB = channelB;
  _pinA = pinA;
  _pinB = pinB;

  ledcSetup(_channelA, _frequency, _resolution);
  ledcSetup(_channelB, _frequency, _resolution);
  ledcAttachPin(_pinA, _channelA);
  ledcAttachPin(_pinB, _channelB);

  _is_init = true;
  return true;
}

void DCPWM::setPWM(float duty_cicle)
{
  _dutyCicle = duty_cicle;
  int maxValue = (1<<_resolution);
  int dc = 0.0;
  // pwm must be in the range from -100.0 to +100.0
    if (_dutyCicle > 100.0)
    {
      _dutyCicle = 100.0;
    }
    else if (_dutyCicle < -100.0)
    {
      _dutyCicle = -100.0;
    }

  if(_dutyCicle>0){
    dc = _dutyCicle * (maxValue/2/100.0) + maxValue/2;
  }
  else if(_dutyCicle<0){
  dc = _dutyCicle * (maxValue/2/100.0) - maxValue/2;
  }
  else if(_dutyCicle==0.0){
  dc = 0.0;
  }
  
  

  if (_is_init)
  {
    // pwm must be in the range from -100.0 to +100.0
    if (dc > maxValue)
    {
      dc = maxValue;
    }
    else if (dc < -maxValue)
    {
      dc = -maxValue;
    }

    if (dc >= 0)
    {
        ledcWrite(_channelA, dc);
        ledcWrite(_channelB, 0);
    }
    else
    {
      ledcWrite(_channelA, 0);
      ledcWrite(_channelB, -dc);
    }
  }
}

float DCPWM::getPWM()
{
  return _dutyCicle;
}