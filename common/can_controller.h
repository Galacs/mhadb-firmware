#pragma once

#include <Arduino.h>
#include <STM32_CAN.h>


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

class CanController {
public:
  CanController();
  void init();
  void send_can(uint32_t id, uint8_t* data, uint8_t data_len);
  
  void send_struct(t_line_sensor_raw_data data);
  
private:
  STM32_CAN m_stm32CAN;
  CAN_message_t m_tx_msg;
};