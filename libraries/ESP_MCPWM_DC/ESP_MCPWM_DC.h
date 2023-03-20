#ifndef ESP_MCPWM_DC_H
#define ESP_MCPWM_DC_H

#include "driver/mcpwm.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

class ESP_MCPWM_DC
{
public:
  ESP_MCPWM_DC()
  {
    is_init = false;
  }
  bool init(int unit, int timer, int pinA, int pinB, int mode);
  void setPWM(float duty_cicle); // duty cicle in +/- percent

private:
  mcpwm_unit_t mcpwm_num;  // number of the MCPWM unit: MCPWM_UNIT_0 or MCPWM_UNIT_1
  mcpwm_timer_t timer_num; // number of the timer within the unit
  bool is_init;
  int pinA;
  int pinB;
  int mode;
};

#endif /* ESP_MCPWM_DC_H */
