#include <Arduino.h>
#include "can_messages.h"
#include <Preferences.h>

uint16_t line_sensors[10] = {0};

void print_line_values(uint16_t* values) {
     for (size_t i = 0; i < 10; i++) {
        Serial.print(values[i]);
        Serial.print('\t');
     }
     Serial.println("");
}

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
    prefs.putFloat("motorLeftZero", 0);
    prefs.putFloat("motorRightZero", 0);

    prefs.putChar("motorLeftDirection", 0);
    prefs.putChar("motorRightDirection", 0);

    prefs.putBool("nvsInit", true);
  }
}

void load_settings() {
  p_motor_left_zero = prefs.getFloat("motorLeftZero");
  p_motor_right_zero = prefs.getFloat("motorRightZero");

  p_motor_left_direction = prefs.getChar("motorLeftDirection");
  p_motor_right_direction = prefs.getChar("motorRightDirection");
}

// Temp test function
void forward(float speed) {
  t_bldc_set_speed data;
  data.motor_id = data.LEFT;
  data.speed = speed;
  can.send_struct(data);
  data.motor_id = data.RIGHT;
  can.send_struct(data);
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
    if (data.motor_id == data.LEFT) {
      prefs.putFloat("motorLeftZero", data.zero_electric_angle);
      prefs.putChar("motorLeftDirection", data.sensor_direction);
    } else {
      prefs.putFloat("motorRightZero", data.zero_electric_angle);
      prefs.putChar("motorRightDirection", data.sensor_direction);
    }
    load_settings();
  }

  static bool update_struct(t_bldc_alignment_settings *data) {
    t_bldc_alignment_settings right;
    right.align_request = right.STORED;
    right.motor_id = right.RIGHT;
    right.zero_electric_angle = p_motor_right_zero;
    right.sensor_direction = p_motor_right_direction;
    controller->send_struct(right);

    data->align_request = data->STORED;
    data->motor_id = data->RIGHT;
    data->zero_electric_angle = p_motor_left_zero;
    data->sensor_direction = p_motor_left_direction;
    return true;
  }

  static void handle_struct(t_line_sensor_raw_data data) {
    if (data.sensor_id == 9) {
      print_line_values(line_sensors);
    }
    line_sensors[data.sensor_id] = data.value;
  }
  static void handle_struct(t_line_sensor_data data) {
    Serial.printf("Line pos: %d\n", data.line_pos);
  }
  

  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};

MHADBCanController<MainCanHandler>* MainCanHandler::controller = nullptr;



CanController<MHADBCanController<MainCanHandler>> can;

void setup() {
  Serial.begin(115200);
  can.init((gpio_num_t)48, (gpio_num_t)34);

  init_settings();
  load_settings();

}

bool done = false;

void loop() {
  // t_line_sensor_raw_data a {.id=10, .value=126};
  // can.send_struct(a);
  // can.send_rtr(CAN_ID::LINE_RAW_SENSOR_DATA);
  can.handle_can();
  if (millis() > 1000 && !done) {
    done = true;
    forward(10.0);
    delay(300);
    forward(0);
  }
  // delay(20);
}
