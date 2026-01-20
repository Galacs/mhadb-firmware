#include <Arduino.h>
#include <FastLED.h>

#include "STM32_CAN.h"

#include <can_messages.h>


#define RGB1 PB12
#define RGB2 PB13

#define BTN1 PB7
#define BTN2 PB6

#define LS0 PA0
#define LS1 PA1
#define LS2 PA2
#define LS3 PA3
#define LS4 PA4
#define LS5 PA5
#define LS6 PA6
#define LS7 PA7
#define LS8 PB0
#define LS9 PB1

class LineCanHandler {
public:
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

  static void handle_struct(t_bldc_alignment_results data) {
    // Serial.print("zero angle: ");
    // Serial.println(data.zero_electric_angle);
    // Serial.printf("direction: %d\n", data.sensor_direction);
  }

  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};

CanController<MHADBCanController<LineCanHandler>> can;

int sensors[] = {LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9 };

CRGB leds_A[5];
CRGB leds_B[5];

uint16_t line_sensors[10];

int init_leds() {
  FastLED.addLeds<WS2812B, PB12>(leds_A, 5);
  FastLED.addLeds<WS2812B, PB13>(leds_B, 5);
}

void update_leds(uint16_t*  values) {
  for (size_t i = 0; i < 10; i++) {
    uint8_t brightness = map(values[i], 0, 1023, 0, 255);
    switch (i)
    {
    case 0:
      leds_A[4] = brightness;
      break;
    case 1:
      leds_A[3] = brightness;
      break;
    case 2:
      leds_A[2] = brightness;
      break;
    case 3:
      leds_A[1] = brightness;
      break;
    case 4:
      leds_A[0] = brightness;
      break;

    case 5:
      leds_B[0] = brightness;
      break;
    case 6:
      leds_B[1] = brightness;
      break;
    case 7:
      leds_B[2] = brightness;
      break;
    case 8:
      leds_B[3] = brightness;
      break;
    case 9:
      leds_B[4] = brightness;
      break;
    }
  }

  FastLED.show();
}

void update_line_sensors(int* sensors, uint16_t*  values) {
  for (size_t i = 0; i < 10; i++) {
    // Devboard sensors
    if (i == 3 /*|| i == 4*/ || i == 5) {
      values[i] = analogRead(sensors[i]);
    } else {
      values[i] = 0;
    }
  }
}

void print_line_values(uint16_t* values) {
     for (size_t i = 0; i < 10; i++) {
        Serial.print(values[i]);
        Serial.print('\t');
     }
     Serial.println("");
}

int16_t get_line_position(uint16_t* values) {
  // TODO: Moyenne réduite
  // const uint8_t weights[] = {4, 14, 24, 34, 44};
  // Devboard
  const uint8_t weights[] = {30, 30, 30, 30, 30};
  int32_t total_moy = 0;
  for (uint8_t i = 0; i < 10; i++) {
    // Serial.println(weights[i % 4] * values[i] * ((i < 4)? 1: -1));
    // TODO: les poids ne sont pas symmétriques
    total_moy += weights[i % 4] * values[i] * ((i < 4)? 1: -1);
  }
  // Serial.printf("Line pos: %d \t", total_moy);
  // Devboard
  // return total_moy/(120);
  return total_moy;
}

void update_can(uint16_t* values) {
  // for (uint8_t i = 0; i < 10; i++) {
  //   t_line_sensor_raw_data msg_content = {i, values[i]};
  //   Can.send_struct(msg_content);
  // }
  t_line_sensor_data msg_content { .line_pos=get_line_position(values) };
  // Serial.printf("can to be line pos: %d\n", msg_content.line_pos);
  can.send_struct(msg_content);

}

// void read_can() {


void setup() {
  Serial.setRx(PB7);
  Serial.setTx(PB6);
  Serial.begin(115200);

  can.init();
  // delay(1000);
  Serial.println("salut rhey");
  can.send_rtr(CAN_ID::BLDC_CURRENT_POS);
  can.send_rtr(CAN_ID::BLDC_CURRENT_SPEED);
  t_bldc_alignment_start align_msg = {.motor_id=t_bldc_alignment_start::LEFT};
  can.send_struct(align_msg);

  // Pins used for serial on devboard
  // pinMode(BTN1, INPUT_PULLUP);
  // pinMode(BTN2, INPUT_PULLUP);

  HardwareTimer* can_timer = new HardwareTimer(TIM2);
  can_timer->setOverflow(120, HERTZ_FORMAT);
  can_timer->attachInterrupt([](){
    can.handle_can();
    update_can(line_sensors);
  });
  // start the timer
  can_timer->resume();

}

void loop() {
  update_line_sensors(sensors, line_sensors);
  print_line_values(line_sensors);
  update_leds(line_sensors);
  // can.handle_can();
  delay(100);
  can.send_rtr(CAN_ID::BLDC_ALIGNMENT_RESULTS);
}