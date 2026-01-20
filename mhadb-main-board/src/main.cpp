#include <Arduino.h>
#include "can_messages.h"


class MainCanHandler
{
  public:
  static void handle_struct(t_bldc_current_pos data) {
    // Serial.printf("Line pos: %d\n", data.shaft_angle);
    // Serial.printf("yay: %f", data.shaft_angle);
    Serial.print("oof: ");
    Serial.println(data.shaft_angle);
    // Serial.println(sizeof(t_bldc_current_pos));
  }
  static void handle_struct(t_bldc_current_speed data) {
    Serial.print("speed: ");
    Serial.println(data.speed);
  }

  static void handle_struct(t_bldc_alignment_results data) {
    Serial.print("zero angle: ");
    Serial.println(data.zero_electric_angle);
    Serial.printf("direction: %d\n", data.sensor_direction);
  }

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
  // t_line_sensor_raw_data a {.id=10, .value=126};
  // can.send_struct(a);
  can.send_rtr(CAN_ID::LINE_RAW_SENSOR_DATA);
  can.handle_can();
  delay(50);
  Serial.println("alive");
}
