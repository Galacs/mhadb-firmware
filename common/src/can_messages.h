#pragma once

#include "can_controller.h"


// To add a new can messages you have to:
// - Define the can message id in the CAN_ID enum with the right order based on the ids (check for duplicates!)
// - Create the can message struct using the CAN_STRUCT(struct_name, can, fields...)
// - Add the correct case macro to handle_can()'s switch

enum class bldc_state_t: uint8_t {
  RESET,
  CALIBRATING,
  RUNNING,
  OFF,
  EMG,
};

enum CAN_ID {
  BUTTON_EVENT = 90,
  BLDC_DISABLE = 100,
  BLDC_SET_SPEED = 110,
  BLDC_ALIGNMENT_START = 120,
  BLDC_CURRENT_POS = 130,
  BLDC_CURRENT_SPEED = 140,
  BLDC_ALIGNMENT_SETTINGS = 150,
  LINE_SENSOR_DATA = 160,
  LINE_RAW_SENSOR_DATA = 170,
  BLDC_STATE = 180,
};

enum motor_id_t: uint8_t {
  RIGHT,
  LEFT,
};

CAN_STRUCT(t_line_sensor_raw_data, CAN_ID::LINE_RAW_SENSOR_DATA,
  uint8_t sensor_id;
  uint16_t mapped_value;
  uint16_t raw_value;
);

enum class line_pos_state_t : uint8_t {
  DETECTED,
  LOST,
  NO_LINE,
  LOSTING,
  T,
  FULL,
  TURN,
};
CAN_STRUCT(t_line_sensor_data, CAN_ID::LINE_SENSOR_DATA,
  int16_t line_pos;
  line_pos_state_t state;
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

CAN_STRUCT(t_bldc_disable, CAN_ID::BLDC_DISABLE,
  motor_id_t motor_id;
);

CAN_STRUCT(t_bldc_state, CAN_ID::BLDC_STATE,
  motor_id_t motor_id;
  bldc_state_t sate;
);

enum class button_id_t: uint8_t {
  LINE_C,
  LINE_D,
};
enum class button_event_kind_t: uint8_t {
  SINGLE,
};
CAN_STRUCT(t_button_event, CAN_ID::BUTTON_EVENT,
  button_id_t button_id;
  button_event_kind_t event_type;
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
      HANDLE_MSG(handler_t, t_bldc_disable);
      HANDLE_MSG(handler_t, t_bldc_state);
      HANDLE_MSG(handler_t, t_button_event);
    }
  };
};

bool elapsed(unsigned long* last_run, unsigned long time) {
  if (millis() > *last_run + time) {
    *last_run = millis();
    return true;
  }
  return false;
}
