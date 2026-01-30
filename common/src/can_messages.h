#pragma once

#include "can_controller.h"


// To add a new can messages you have to:
// - Define the can message id in the CAN_ID enum with the right order based on the ids (check for duplicates!)
// - Create the can message struct using the CAN_STRUCT(struct_name, can, fields...)
// - Add the correct case macro to handle_can()'s switch


enum CAN_ID {
  LINE_SENSOR_DATA = 0x40,
  LINE_RAW_SENSOR_DATA = 0x50,
  BLDC_ALIGNMENT_START = 0x60,
  BLDC_CURRENT_POS = 0x70,
  BLDC_CURRENT_SPEED = 0x80,
  BLDC_ALIGNMENT_SETTINGS = 0x90,
  BLDC_SET_SPEED = 0xA0,
};

enum motor_id_t: int8_t {
  RIGHT,
  LEFT,
};

CAN_STRUCT(t_line_sensor_raw_data, CAN_ID::LINE_RAW_SENSOR_DATA,
  uint8_t sensor_id;
  uint16_t value;
);

CAN_STRUCT(t_line_sensor_data, CAN_ID::LINE_SENSOR_DATA,
  int16_t line_pos;
);

CAN_STRUCT(t_bldc_current_pos, CAN_ID::BLDC_CURRENT_POS,
  motor_id_t motor_id;
  float shaft_angle;
);

CAN_STRUCT(t_bldc_current_speed, CAN_ID::BLDC_CURRENT_SPEED,
  motor_id_t motor_id;
  float speed;
);

CAN_STRUCT(t_bldc_alignment_start, CAN_ID::BLDC_ALIGNMENT_START,
  motor_id_t motor_id;
);

CAN_STRUCT(t_bldc_alignment_settings, CAN_ID::BLDC_ALIGNMENT_SETTINGS,
  motor_id_t motor_id;
  enum align_request_t: int8_t {
    STORED,
    CALIBRATED,
  } align_request;
  float zero_electric_angle;
  int8_t sensor_direction;
);

CAN_STRUCT(t_bldc_set_speed, CAN_ID::BLDC_SET_SPEED,
  motor_id_t motor_id;
  float speed;
);

template <typename handler_t>
class MHADBCanController : public CanController<MHADBCanController<handler_t>> {
public:
  handler_t handler;
  MHADBCanController() {
    // Set the static controller reference
    handler_t::setController(this);
  }

  void handle_can_special(t_can_frame *frame) {
    switch (frame->id) {
      HANDLE_MSG(handler_t, t_line_sensor_raw_data);
      HANDLE_MSG(handler_t, t_line_sensor_data);
      HANDLE_MSG(handler_t, t_bldc_current_pos);
      HANDLE_MSG(handler_t, t_bldc_current_speed);
      HANDLE_MSG(handler_t, t_bldc_alignment_start);
      HANDLE_MSG(handler_t, t_bldc_alignment_settings);
      HANDLE_MSG(handler_t, t_bldc_set_speed);
    }
  };
};