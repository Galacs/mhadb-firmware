#include <Arduino.h>
#include <FastLED.h>

#include "STM32_CAN.h"

#include <can_controller.h>


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

class LineCanController: public CanController {};

LineCanController Can;

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
  Serial.printf("Line pos: %d \t", total_moy);
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
  Can.send_struct(msg_content);

  // Decode example
  // t_line_sensor_raw_data msg_content;
  // memcpy(&msg_content, CAN_TX_msg.buf, sizeof(msg_content));
  // Serial.println(msg_content.value);

  // Can.write(CAN_TX_msg);
}

// void read_can() {
//   if (Can.read(CAN_RX_msg)) {
//     Serial.print("received id: ");
//     Serial.println(CAN_RX_msg.id);
//   }
// }

void setup() {
  Serial.setRx(PB7);
  Serial.setTx(PB6);
  Serial.begin(115200);

  Can.init();
  // delay(1000);
  Serial.println("salut rhey");

  // put your setup code here, to run once:
  // Pins used for serial on devboard
  // pinMode(BTN1, INPUT_PULLUP);
  // pinMode(BTN2, INPUT_PULLUP);

}

void loop() {
  update_line_sensors(sensors, line_sensors);
  print_line_values(line_sensors);
  update_can(line_sensors);
  update_leds(line_sensors);
  // read_can();
  delay(50);
}