#pragma once

#include <Arduino.h>

#ifdef ARDUINO_ARCH_STM32
#include "STM32_CAN.h"
#endif


// To add a new can messages you have to:
// - Define the can message id in the CAN_ID enum with the right order based on the ids
// - Create the packed struct that can't exceed 8 bytes
// - Add the send_struct function with the right can id and size
// - Add the handle_struct function definition and default implementation {}
// - Add the correct case to handle_can()'s switch

enum CAN_ID {
  LINE_SENSOR_DATA = 0x40,
  LINE_RAW_SENSOR_DATA = 0x50,
};

struct __attribute__ ((packed)) t_line_sensor_raw_data {
  uint8_t id;
  uint16_t  value;
};

struct __attribute__ ((packed)) t_line_sensor_data {
  int16_t line_pos;
};

struct t_can_frame {
  uint32_t id = 0;
  uint8_t buf[8] = { 0 };
  uint8_t len = 8;
};

class CanController {
public:
  CanController();
  void init();
  void send_can(uint32_t id, uint8_t* data, uint8_t data_len);
  void receive_can(t_can_frame* frame);

  void handle_can();
  
  void send_struct(t_line_sensor_raw_data data);
  void send_struct(t_line_sensor_data data);

  virtual void handle_struct(t_line_sensor_raw_data data) {};
  virtual void handle_struct(t_line_sensor_data data) {};

private:
#ifdef ARDUINO_ARCH_STM32
  STM32_CAN m_stm32CAN;
  CAN_message_t m_tx_msg;
  CAN_message_t m_rx_msg;
#endif
};