#include <Arduino.h>
#include <FastLED.h>

#include "STM32_CAN.h"

#include "can_messages.h"


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

STM32_CAN Can(CAN1, DEF) ;
static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_RX_msg;

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
    values[i] = analogRead(sensors[i]);
  }
}

void init_can() {
  Can.begin();
  Can.setBaudRate(1000000);
  Can.enableLoopBack();
}

void update_can(uint16_t* values) {

  for (size_t i = 0; i < 10; i++) {
    // Struct
    t_line_sensor_data msg_content = {(uint8_t) i, values[i]};
    CAN_TX_msg.len = sizeof(msg_content);
    memcpy(CAN_TX_msg.buf, &msg_content, sizeof(msg_content));


    // Manual
    CAN_TX_msg.id = (0x69);
    // CAN_TX_msg.len = 8;
    // CAN_TX_msg.buf[0] =  i;
    // CAN_TX_msg.buf[1] =  ((uint16_t) values[i] >> 16);
    // CAN_TX_msg.buf[2] =  (uint16_t) values[i];
    // CAN_TX_msg.buf[3] =  0;
    // CAN_TX_msg.buf[4] =  0;
    // CAN_TX_msg.buf[5] =  0;
    // CAN_TX_msg.buf[6] =  0;
    // CAN_TX_msg.buf[7] =  0;
  }
  t_line_sensor_data msg_content;
  memcpy(&msg_content, CAN_TX_msg.buf, sizeof(msg_content));
  //uint32_t var3 = (CAN_TX_msg.buf[1] << 16) +  CAN_TX_msg.buf[2];
  Serial.println(msg_content.value);

  Can.write(CAN_TX_msg);

}

void print_line_values(uint16_t* values) {
     for (size_t i = 0; i < 10; i++) {
        Serial.print(values[i]);
        Serial.print(" ");
     }
     Serial.println("");
}

void read_can() {
  if (Can.read(CAN_RX_msg)) {
    Serial.print("received id: ");
    Serial.println(CAN_RX_msg.id);
  }
}

void setup() {
  Serial.begin(115200);
  init_can();

  // put your setup code here, to run once:
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

}

void loop() {
  update_line_sensors(sensors, line_sensors);
  print_line_values(line_sensors);
  update_can(line_sensors);
  update_leds(line_sensors);
  read_can();
  delay(1000);
  // put your main code here, to run repeatedly:
}