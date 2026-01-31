#include <Arduino.h>
#include "can_messages.h"
#include <Preferences.h>
#include <Commander.h>

uint16_t line_sensors[10] = {0};

enum bldc_main_t: uint8_t {
  RESET,
  INITIALIZING,
  ARMED,
  FOLLOWING,
  EMG,
};

void print_line_values(uint16_t* values) {
     for (size_t i = 0; i < 10; i++) {
        Serial.print(values[i]);
        Serial.print('\t');
     }
     Serial.println("");
}

Commander command = Commander(Serial);

// Temp settings
Preferences prefs;
float p_motor_left_zero;
int8_t p_motor_left_direction;
float p_motor_right_zero;
int8_t p_motor_right_direction;

void init_settings() {
  prefs.begin("mainPrefs");
  bool tpInit = prefs.isKey("nvsInit");
  if (tpInit == false) {
    prefs.putFloat("motorLZero", 0);
    prefs.putFloat("motorRZero", 0);

    prefs.putChar("motorLDir", 0);
    prefs.putChar("motorRDir", 0);

    prefs.putBool("nvsInit", true);
  }
}

void load_settings() {
  p_motor_left_zero = prefs.getFloat("motorLZero");
  p_motor_right_zero = prefs.getFloat("motorRZero");

  p_motor_left_direction = prefs.getChar("motorLDir");
  p_motor_right_direction = prefs.getChar("motorRDir");
}

class MainCanHandler
{
  public:
  static MHADBCanController<MainCanHandler>* controller; 
  
  static void setController(MHADBCanController<MainCanHandler>* ctrl) {
    controller = ctrl;
  }

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

  static void handle_struct(t_bldc_alignment_settings data) {
    Serial.print("zero angle: ");
    Serial.println(data.zero_electric_angle);
    Serial.printf("direction: %d\n", data.sensor_direction);

    if (data.align_request != data.CALIBRATED)
      return;
    if (data.motor_id == motor_id_t::LEFT) {
      prefs.putFloat("motorLZero", data.zero_electric_angle);
      prefs.putChar("motorLDir", data.sensor_direction);
    } else {
      prefs.putFloat("motorRZero", data.zero_electric_angle);
      prefs.putChar("motorRDir", data.sensor_direction);
    }
    load_settings();
    Serial.printf("Got: %f, %d", data.zero_electric_angle, data.sensor_direction);
  }

  static bool update_struct(t_bldc_alignment_settings *data) {
    t_bldc_alignment_settings right;
    right.align_request = right.STORED;
    right.motor_id = motor_id_t::RIGHT;
    right.zero_electric_angle = p_motor_right_zero;
    right.sensor_direction = p_motor_right_direction;
    controller->send_struct(right);

    data->align_request = data->STORED;
    data->motor_id = motor_id_t::RIGHT;
    data->zero_electric_angle = p_motor_left_zero;
    data->sensor_direction = p_motor_left_direction;
    Serial.println("sent saved settings");
    return true;
  }

  static void handle_struct(t_line_sensor_raw_data data) {
    if (data.sensor_id == 9) {
      // print_line_values(line_sensors);
    }
    line_sensors[data.sensor_id] = data.value;
  }
  static void handle_struct(t_line_sensor_data data) {
    // Serial.printf("Line pos: %d\n", data.line_pos);
  }
  

  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};

MHADBCanController<MainCanHandler>* MainCanHandler::controller = nullptr;
CanController<MHADBCanController<MainCanHandler>> can;

void doStartAlign(char *cmd) {
  // command.scalar(&debugValue, cmd);
  // Serial.println(debugValue);
  Serial.println("test");
  t_bldc_alignment_start msg;
  msg.motor_id = motor_id_t::LEFT; // not used by bldc 
  can.send_struct(msg);
  // can.send_rtr(BLDC_ALIGNMENT_SETTINGS);
}

void doSendAlign(char *cmd) {
  // command.scalar(&debugValue, cmd);
  // Serial.println(debugValue);
  t_bldc_alignment_settings right;
  right.align_request = right.STORED;
  right.motor_id = motor_id_t::RIGHT;
  right.zero_electric_angle = p_motor_right_zero;
  right.sensor_direction = p_motor_right_direction;
  can.send_struct(right);

  t_bldc_alignment_settings left;
  left.align_request = left.STORED;
  left.motor_id = motor_id_t::LEFT;
  left.zero_electric_angle = p_motor_left_zero;
  left.sensor_direction = p_motor_left_direction;
  can.send_struct(left);
  Serial.println("sent saved settings");
}

// Temp test function
void forward(float speed) {
  t_bldc_set_speed data;
  data.motor_id = motor_id_t::LEFT;
  data.speed = speed;
  can.send_struct(data);
  data.motor_id = motor_id_t::RIGHT;
  can.send_struct(data);
}

void doSendForward(char *cmd) {
  float target = 0.0;
  command.scalar(&target, cmd);
  forward(target);
  Serial.printf("forward: %f\n", target);
}

void setup() {
  Serial.begin(115200);
  // can.init((gpio_num_t)48, (gpio_num_t)34);
  can.init((gpio_num_t)10, (gpio_num_t)9);
  
  // Commander
  command.add('A', doStartAlign);
  command.add('B', doSendAlign);
  command.add('F', doSendForward);

  init_settings();
  load_settings();

}

bool done = false;

void loop() {
  // t_line_sensor_raw_data a {.id=10, .value=126};
  // can.send_struct(a);
  // can.send_rtr(CAN_ID::LINE_RAW_SENSOR_DATA);
  can.handle_can();
  // if (millis() > 10000 && !done) {
  //   done = true;
  //   forward(5.0);
  //   Serial.println("gooo");
  //   delay(5000);
  //   forward(0.0);
  // }
  command.run();
  // delay(20);
}
