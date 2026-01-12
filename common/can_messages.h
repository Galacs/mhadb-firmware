#pragma once

#include <Arduino.h>

struct __attribute__ ((packed)) t_line_sensor_data {
  uint8_t id;
  uint16_t  value;
};
