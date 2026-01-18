#pragma once

#include <Arduino.h>

#ifdef ARDUINO_ARCH_STM32
#include "STM32_CAN.h"
#endif
#ifdef ARDUINO_ARCH_ESP32
#include "driver/twai.h"
#endif


// To add a new can messages you have to:
// - Define the can message id in the CAN_ID enum with the right order based on the ids
// - Create the packed struct that can't exceed 8 bytes
// - Add the send_struct function with the right can id and size
// - Add the handle_struct function definition and default implementation {}
// - Add the update_struct function definition and default implementation {}
// - Add the correct case to handle_can()'s switch

enum CAN_ID {
  LINE_SENSOR_DATA = 0x40,
  LINE_RAW_SENSOR_DATA = 0x50,
  BLDC_CURRENT_POS = 0x60,
  BLDC_CURRENT_SPEED = 0x70,
};

struct __attribute__ ((packed)) t_line_sensor_raw_data {
  uint8_t id;
  uint16_t  value;
};

struct __attribute__ ((packed)) t_line_sensor_data {
  int16_t line_pos;
};

struct __attribute__ ((packed)) t_bldc_current_pos {
  enum motor_id_t {
    RIGHT,
    LEFT,
  } motor_id;
  float shaft_angle;
};

struct __attribute__ ((packed)) t_bldc_current_speed {
  enum motor_id_t {
    RIGHT,
    LEFT,
  } motor_id;
  float speed;
};

struct t_can_frame {
  uint32_t id = 0;
  uint8_t buf[8] = { 0 };
  uint8_t len = 0;
  bool rtr = false;
};

class CanController {
public:
  CanController();
#ifdef ARDUINO_ARCH_STM32
  void init();
#endif
#ifdef ARDUINO_ARCH_ESP32
  void init(gpio_num_t rx, gpio_num_t tx);
#endif

  void send_can(t_can_frame frame);
  bool receive_can(t_can_frame* frame);

  void handle_can();
  
  void send_rtr(CAN_ID msg_id);

  void send_struct(t_line_sensor_raw_data data);
  void send_struct(t_line_sensor_data data);
  void send_struct(t_bldc_current_pos data);
  void send_struct(t_bldc_current_speed data);

  virtual void handle_struct(t_line_sensor_raw_data data) {};
  virtual void handle_struct(t_line_sensor_data data) {};
  virtual void handle_struct(t_bldc_current_pos data) {};
  virtual void handle_struct(t_bldc_current_speed data) {};

  virtual bool update_struct(t_line_sensor_raw_data* data) {return false;};
  virtual bool update_struct(t_line_sensor_data* data) {return false;};
  virtual bool update_struct(t_bldc_current_pos* data) {return false;};
  virtual bool update_struct(t_bldc_current_speed* data) {return false;};

private:
#ifdef ARDUINO_ARCH_STM32
  STM32_CAN m_stm32CAN;
  CAN_message_t m_tx_msg;
  CAN_message_t m_rx_msg;
#endif
};