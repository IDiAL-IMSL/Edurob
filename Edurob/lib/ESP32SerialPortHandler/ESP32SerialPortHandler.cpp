#include <Dynamixel2Arduino.h>

class ESP32SerialPortHandler : public DYNAMIXEL::SerialPortHandler
{
  public:
    ESP32SerialPortHandler(HardwareSerial& port, int rx_pin = -1, int tx_pin = -1, const int dir_pin = -1)
      : SerialPortHandler(port, dir_pin), port_(port), dir_pin_(dir_pin)
    {
      rx_pin_ = rx_pin;
      tx_pin_ = tx_pin;
    }

    virtual void begin(unsigned long baud) override
    {
      baud_ = baud;
      if ( getOpenState() ) {
        // Already open, so just update baud rate.
        port_.updateBaudRate(baud_);
      } else {
        // If port_.begin(...) were called again, esp32 could lock up.
        port_.begin(baud_, SERIAL_8N1, rx_pin_, tx_pin_);
      }

      // From robitis example code.  Set default RX/TX direction.
      if (dir_pin_ != -1) {
        pinMode(dir_pin_, OUTPUT);
        digitalWrite(dir_pin_, LOW);
        while (digitalRead(dir_pin_) != LOW);
      }

      setOpenState(true);
    }

  private:
    HardwareSerial& port_;
    unsigned long baud_;
    const int dir_pin_;
    int rx_pin_;
    int tx_pin_;
};
