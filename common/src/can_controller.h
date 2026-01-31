#pragma once

#include <Arduino.h>

#ifdef ARDUINO_ARCH_STM32
#include "STM32_CAN.h"
#endif
#ifdef ARDUINO_ARCH_ESP32
#include "driver/twai.h"
#endif


struct t_can_frame {
  uint32_t id = 0;
  uint8_t buf[8] = { 0 };
  uint8_t len = 0;
  bool rtr = false;
};

#define CAN_STRUCT(struct_name, can_id, ...) \
  struct __attribute__((packed)) struct_name { \
    static constexpr CAN_ID ID = can_id; \
    __VA_ARGS__ \
  }; \
  static_assert(sizeof(struct_name) <= 8, #struct_name " exceeds CAN frame size (8 bytes)"); \
  static_assert(std::is_trivially_copyable<struct_name>::value, #struct_name " must be trivially copyable") \

#define HANDLE_MSG(HANDLER_T, MSG_T) \
  case MSG_T::ID: { \
      MSG_T data; \
      if (frame->rtr){ \
          if (HANDLER_T::update_struct(&data)) { \
              this->send_struct(data); \
          }; \
      } \
      else { \
          memcpy(&data, frame->buf, sizeof(data)); \
          HANDLER_T::handle_struct(data); \
      } \
      break; \
  } \

template<typename Child>
class CanController {
public:
#ifdef ARDUINO_ARCH_STM32
  CanController(): m_stm32CAN(CAN1, DEF) {};
  void init() {
    m_stm32CAN.begin();
    m_stm32CAN.setBaudRate(100000);
  };

  void send_can(t_can_frame frame) {
    CAN_message_t tx_msg;
    tx_msg.id = frame.id;
    tx_msg.len = frame.len;
    tx_msg.flags.remote = frame.rtr;
    // Serial.printf("data len: %d\n", data_len);
    memcpy(&tx_msg.buf, frame.buf, frame.len);
    if (m_stm32CAN.write(tx_msg)) {
      // Serial.println("can sent");
    };
  };

  bool receive_can(t_can_frame* frame) {
    CAN_message_t rx_msg;
    if (!m_stm32CAN.read(rx_msg)){
        // Serial.println("no can frames");
        return false;
    }

    frame->id = rx_msg.id;
    frame->len = rx_msg.len;
    frame->rtr = rx_msg.flags.remote;
    memcpy(frame->buf, &rx_msg.buf, rx_msg.len);
    return true;
  }
#endif
#ifdef ARDUINO_ARCH_ESP32
  CanController() {};
  void init(gpio_num_t rx, gpio_num_t tx) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("Driver installed");
    } else {
        Serial.println("Failed to install driver");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        Serial.println("Driver started");
    } else {
        Serial.println("Failed to start driver");
        return;
    }
  };

  void send_can(t_can_frame frame) {
      twai_message_t message;
      message.identifier = frame.id;
      message.data_length_code = frame.len;
      // Serial.printf("data len: %d\n", frame.len);
      memcpy(&message.data, frame.buf, frame.len);
      if (twai_transmit(&message, 0) == ESP_OK) {
          // printf("Message queued for transmission\n");
      } else {
          printf("Failed to queue message for transmission\n");
      }
  };

  bool receive_can(t_can_frame* frame) {
      //uint32_t alerts_triggered;
      //twai_read_alerts(&alerts_triggered, 0);
      twai_message_t message;
      if (twai_receive(&message, 0) == ESP_ERR_TIMEOUT) {
          // Serial.println("no can frames");
          return false;
      }
      frame->rtr = message.rtr;
      frame->len = message.data_length_code;
      frame->id = message.identifier;
      memcpy(frame->buf, &message.data, message.data_length_code);
      return true;
  };

#endif
  
  void send_rtr(uint32_t msg_id) {
    t_can_frame frame {};
    frame.id = msg_id;
    frame.rtr = true;
    send_can(frame);
  };

  template<typename T>
  void send_struct(T data) {
    t_can_frame frame {};
    frame.id = T::ID;
    frame.len = sizeof(data);
    memcpy(&frame.buf, &data, frame.len);
    send_can(frame);
  }

  void handle_can() {
    t_can_frame frame;
    if (!receive_can(&frame)) {
        return;
    }
    // Serial.printf("The can id: %d\n", frame.id);
    static_cast<Child*>(this)->handle_can_special(&frame);
  }

private:
#ifdef ARDUINO_ARCH_STM32
  STM32_CAN m_stm32CAN;
#endif
};