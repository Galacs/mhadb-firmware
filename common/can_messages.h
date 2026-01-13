#pragma once

#include <Arduino.h>

enum CAN_ID {
  LINE_RAW_SENSOR_DATA = 0x50,
  LINE_SENSOR_DATA = 0x40,
};

struct __attribute__ ((packed)) t_line_sensor_raw_data {
  uint8_t id;
  uint16_t  value;
};

struct __attribute__ ((packed)) t_line_sensor_data {
  int16_t line_pos;
};

