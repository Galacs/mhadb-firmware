#include <Arduino.h>
#include "can_controller.h"

class MainCanController: public CanController
{
public:
  void handle_struct(t_line_sensor_raw_data data) {
    Serial.println("sdfsdsdf received");
  }

};

MainCanController can;

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
