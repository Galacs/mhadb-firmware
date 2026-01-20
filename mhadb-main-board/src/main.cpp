#include <Arduino.h>
#include "can_messages.h"


class LineCanHandler
{
  public:

  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};


CanController<MHADBCanController<MainCanHandler>> can;

void setup() {
  Serial.begin(115200);
  can.init((gpio_num_t)12, (gpio_num_t)14);

}

void loop() {
  t_line_sensor_raw_data a {.id=10, .value=126};
  can.send_struct(a);
  delay(1000);
  Serial.println("alive");
}
