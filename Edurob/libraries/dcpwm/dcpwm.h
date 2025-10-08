#ifndef DCPWM_H
#define DCPWM_H

#include "Arduino.h"

class DCPWM
{
public:
  DCPWM()
  {
    _is_init = false;
  }
  bool init(int frequency, int resolution, int channelA,int channelB,  int pinA, int pinB);
  void setPWM(float duty_cicle); // duty cicle in +/- percent
  float getPWM(); // returns current duty cicle in +/- percent

private:
  int _frequency;
  int _resolution;
  int _channelA;
  int _channelB;
  bool _is_init;
  int _pinA;
  int _pinB;
  int _dutyCicle;
};

#endif /* DCPWM_H */
