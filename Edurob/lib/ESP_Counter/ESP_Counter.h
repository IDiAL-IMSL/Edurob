#ifndef ESP_COUNTER_H
#define ESP_COUNTER_H

//#include <driver/pcnt.h>
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"

class ESP_Counter {
  public:
    ESP_Counter () {
      is_init = false;
    }
    bool init(int unit, int pinA, int pinB);
    int64_t getCount();
    bool setFilter(uint16_t param);
    void resetCounter();
  
  private:
    static pcnt_isr_handle_t user_isr_handle; //ISR service handle
    static bool isr_is_init;

    int16_t r_enc_count;
    pcnt_unit_t pcnt_unit;
    bool is_init;

};


#endif /* ESP_COUNTER_H */

