#include "ESP_MCPWM_DC.h"

/* ESP32 has 2 Motorcontroller PWM units with 3 timers, 3 operators and 6 PWM outputs
*/

bool ESP_MCPWM_DC::init(int unit_, int timer_, int pinA_, int pinB_, int mode_)
{ 

  
                                     //mode => 0=2ch-pwm | 1=pwm(A)&dir(B)
  mcpwm_num = (mcpwm_unit_t)unit_;   // number of the MCPWM unit: MCPWM_UNIT_0 or MCPWM_UNIT_1
  timer_num = (mcpwm_timer_t)timer_; // number of the timer within the unit
  pinA = pinA_;
  pinB = pinB_;
  mode = mode_;
  is_init = false;

  if (mcpwm_gpio_init(mcpwm_num, mcpwm_io_signals_t(2 * timer_num + MCPWM0A), pinA) != ESP_OK)
    return false;
  if (mode == 0)
  {
    if (mcpwm_gpio_init(mcpwm_num, mcpwm_io_signals_t(2 * timer_num + MCPWM0B), pinB) != ESP_OK)
      return false;
  }
  else
  {
    gpio_config_t io_out_conf = {
        .pin_bit_mask = (1ULL << pinB),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = gpio_pullup_t(0),
        .pull_down_en = gpio_pulldown_t(0),
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_out_conf);
  }

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 25000; //frequency = 30 kHz,
  pwm_config.cmpr_a = 0;        //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;        //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  if (mcpwm_init(mcpwm_num, timer_num, &pwm_config) != ESP_OK)
    return false; //Configure PWM0A & PWM0B with above settings
  mcpwm_start(mcpwm_num,timer_num);
  is_init = true;
  return true;
}

void ESP_MCPWM_DC::setPWM(float duty_cicle)
{
  if (is_init)
  {
    // pwm must be in the range from -100.0 to +100.0
    if (duty_cicle > 100.0)
    {
      duty_cicle = 100.0;
    }
    else if (duty_cicle < -100.0)
    {
      duty_cicle = -100.0;
    }

    if (duty_cicle == 0.0)
    {
      mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_GEN_A);
      if (mode == 0)
      {
        mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_GEN_B);
      }
    }
    else if (duty_cicle > 0.0)
    {
      if (mode == 0)
      {
        mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_GEN_B);
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_GEN_A, duty_cicle);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
      }
      else
      {
        gpio_set_level(gpio_num_t(pinB), 0);
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_GEN_A, duty_cicle);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
      }
    }
    else
    {
      if (mode == 0)
      {
        mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_GEN_A);
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_GEN_B, -duty_cicle);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_GEN_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
      }
      else
      {
        gpio_set_level(gpio_num_t(pinB), 1);
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_GEN_A, -duty_cicle);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
      }
    }
  }
}
