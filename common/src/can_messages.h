#pragma once

#include "can_controller.h"


// To add a new can messages you have to:
// - Define the can message id in the CAN_ID enum with the right order based on the ids
// - Create the packed struct containing the CAN_ID as a static constexpr that can't exceed 8 bytes
// - Add the correct case macro to handle_can()'s switch

enum CAN_ID {
  LINE_SENSOR_DATA = 0x40,
  LINE_RAW_SENSOR_DATA = 0x50,
  BLDC_CURRENT_POS = 0x60,
  BLDC_CURRENT_SPEED = 0x70,
  BLDC_ALIGNEMENT_START = 0x80,
};

struct __attribute__ ((packed)) t_line_sensor_raw_data {
  static constexpr CAN_ID ID = LINE_RAW_SENSOR_DATA;
  uint8_t id;
  uint16_t  value;
};

struct __attribute__ ((packed)) t_line_sensor_data {
  static constexpr CAN_ID ID = LINE_SENSOR_DATA;
  int16_t line_pos;
};

struct __attribute__ ((packed)) t_bldc_current_pos {
  static constexpr CAN_ID ID = BLDC_CURRENT_POS;
  enum motor_id_t {
    RIGHT,
    LEFT,
  } motor_id;
  float shaft_angle;
};

struct __attribute__ ((packed)) t_bldc_current_speed {
  static constexpr CAN_ID ID = BLDC_CURRENT_SPEED;
  enum motor_id_t {
    RIGHT,
    LEFT,
  } motor_id;
  float speed;
};

struct __attribute__ ((packed)) t_bldc_alignement_start {
  static constexpr CAN_ID ID = BLDC_ALIGNEMENT_START;
  enum motor_id_t {
    RIGHT,
    LEFT,
  } motor_id;
};

template <typename handler_t>
class MHADBCanController: public CanController<MHADBCanController<handler_t>> {
  public:
  void handle_can_special(t_can_frame *frame) {
    switch (frame->id) {
      HANDLE_MSG(handler_t, t_line_sensor_raw_data);
      HANDLE_MSG(handler_t, t_line_sensor_data);
      HANDLE_MSG(handler_t, t_bldc_current_pos);
      HANDLE_MSG(handler_t, t_bldc_current_speed);
      HANDLE_MSG(handler_t, t_bldc_alignement_start);
    }
  };
};