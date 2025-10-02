#include "ESP_Counter.h"
//#include <Arduino.h>
#if 0
typedef struct {
    int pulse_gpio_num;             /*!< Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored */
    int ctrl_gpio_num;              /*!< Control signal input GPIO number, a negative value will be ignored */
    pcnt_ctrl_mode_t lctrl_mode;    /*!< PCNT low control mode */
    pcnt_ctrl_mode_t hctrl_mode;    /*!< PCNT high control mode */
    pcnt_count_mode_t pos_mode;     /*!< PCNT positive edge count mode */
    pcnt_count_mode_t neg_mode;     /*!< PCNT negative edge count mode */
    int16_t counter_h_lim;          /*!< Maximum counter value */
    int16_t counter_l_lim;          /*!< Minimum counter value */
    pcnt_unit_t unit;               /*!< PCNT unit number */
    pcnt_channel_t channel;         /*!< the PCNT channel */
} pcnt_config_t;
#endif

static int64_t longcounter[PCNT_UNIT_MAX];

typedef struct {
  int unit;         // the PCNT unit that originated an interrupt
  uint32_t status;  // information on the event type that caused the interrupt
} pcnt_evt_t;

static void IRAM_ATTR pcnt_intr_handler(void *arg) {
  int unit = (int)arg;
  pcnt_evt_t evt;
  evt.unit = unit;
  pcnt_get_event_status((pcnt_unit_t)unit, &evt.status);
  if (evt.status == PCNT_EVT_L_LIM) {
    longcounter[unit] += INT16_MIN;
    //ESP_LOGI(TAG, "Underflow of counter unit %d",pcnt_unit);
  }
  if (evt.status == PCNT_EVT_H_LIM) {
    longcounter[unit] += INT16_MAX;
    //ESP_LOGI(TAG, "Overflow of counter unit %d",pcnt_unit);
  }
  //   uint32_t intr_status = PCNT.int_st.val;
  //   for (int i = 0; i < PCNT_UNIT_MAX; i++) {
  //     if (intr_status & (BIT(i))) {
  //       uint32_t status = PCNT.status_unit[i].val;
  //       PCNT.int_clr.val = BIT(i);
  //       if (status & PCNT_EVT_L_LIM) {
  //         longcounter[i] += INT16_MIN;
  // //        Serial.println("underflow");
  //         }
  //       if (status & PCNT_EVT_H_LIM) {
  //         longcounter[i] += INT16_MAX;
  // //        Serial.println("overflow");
  //       }
  //     }
  //   }
}

/* ESP32 has 8 counter unit with 2 channels
*/

bool ESP_Counter::init(int unit, int pinA, int pinB) {
  pcnt_config_t r_enc_config;
  pcnt_unit = (pcnt_unit_t)unit;
  is_init = false;

  /* quadrature (4x) counting of encoder singnals
       using 2 channels of 1 unit */

  /* channel 0: pulse A, ctrl B */
  r_enc_config.pulse_gpio_num = (gpio_num_t)pinA;  //Rotary Encoder Chan A
  r_enc_config.ctrl_gpio_num = (gpio_num_t)pinB;   //Rotary Encoder Chan B
  r_enc_config.unit = pcnt_unit;
  r_enc_config.channel = PCNT_CHANNEL_0;   //
  r_enc_config.pos_mode = PCNT_COUNT_INC;  //Count + On Rising-Edges of A
  r_enc_config.neg_mode = PCNT_COUNT_DEC;  // count - on falling edge of A

  r_enc_config.lctrl_mode = PCNT_MODE_KEEP;     // Rising A on HIGH B = CW Step
  r_enc_config.hctrl_mode = PCNT_MODE_REVERSE;  // Rising A on LOW B = CCW Step
  r_enc_config.counter_h_lim = INT16_MAX;
  r_enc_config.counter_l_lim = INT16_MIN;  //-INT16_MAX ;

  if (pcnt_unit_config(&r_enc_config) != ESP_OK) return false;

  /* channel 1: pulse B, ctrl A*/
  r_enc_config.pulse_gpio_num = (gpio_num_t)pinB;  //Rotary Encoder Chan A
  r_enc_config.ctrl_gpio_num = (gpio_num_t)pinA;   //Rotary Encoder Chan B
  r_enc_config.unit = pcnt_unit;
  r_enc_config.channel = PCNT_CHANNEL_1;   //
  r_enc_config.pos_mode = PCNT_COUNT_DEC;  //Count - on Rising-Edges of B
  r_enc_config.neg_mode = PCNT_COUNT_INC;  // count + on falling edge of B

  r_enc_config.lctrl_mode = PCNT_MODE_KEEP;     // Rising A on HIGH B = CW Step
  r_enc_config.hctrl_mode = PCNT_MODE_REVERSE;  // Rising A on LOW B = CCW Step

  if (pcnt_unit_config(&r_enc_config) != ESP_OK) return false;

  if (pcnt_set_filter_value(pcnt_unit, 100) != ESP_OK) return false;  // 4096 / U -> ~500.000 1/s -> 0,5 MHz-> 160 cycles of 80 MHz clock
  if (pcnt_filter_enable(pcnt_unit) != ESP_OK) return false;

  /* Enable events on maximum and minimum limit values (over and underflow) */
  if (pcnt_event_enable(pcnt_unit, PCNT_EVT_H_LIM) != ESP_OK) return false;
  if (pcnt_event_enable(pcnt_unit, PCNT_EVT_L_LIM) != ESP_OK) return false;

  if (gpio_pulldown_en((gpio_num_t)pinA) != ESP_OK) return false;
  if (gpio_pulldown_en((gpio_num_t)pinB) != ESP_OK) return false;

  if (pcnt_counter_pause(pcnt_unit) != ESP_OK) return false;  // Initial PCNT init
  if (pcnt_counter_clear(pcnt_unit) != ESP_OK) return false;

  /* Register ISR handler and enable interrupts for PCNT unit */
  if (!isr_is_init) {
    pcnt_isr_service_install(0);
    //if (pcnt_isr_handler_add(pcnt_unit, pcnt_intr_handler, (void *)pcnt_unit) != ESP_OK) return false;
    //if (pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle) != ESP_OK) return false;
    isr_is_init = true;
  }
  if (pcnt_isr_handler_add(pcnt_unit, pcnt_intr_handler, (void *)pcnt_unit) != ESP_OK) return false;
  if (pcnt_intr_enable(pcnt_unit) != ESP_OK) return false;
  if (pcnt_counter_resume(pcnt_unit) != ESP_OK) return false;

  is_init = true;
  return true;
}

int64_t ESP_Counter::getCount() {
  if (is_init) {
    pcnt_get_counter_value(pcnt_unit, &r_enc_count);
    int64_t count = longcounter[pcnt_unit] + r_enc_count;
    return count;
  }
  return 0;
}


bool ESP_Counter::setFilter(uint16_t param) {
  if (pcnt_set_filter_value(pcnt_unit, param) != ESP_OK) return false;
  if (pcnt_filter_enable(pcnt_unit) != ESP_OK) return false;
  return true;
}

void ESP_Counter::resetCounter() {
  pcnt_counter_clear(pcnt_unit);
}

pcnt_isr_handle_t ESP_Counter::user_isr_handle = nullptr;
bool ESP_Counter::isr_is_init = false;