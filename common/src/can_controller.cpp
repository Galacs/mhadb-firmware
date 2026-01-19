#include "can_controller.h"
#include <Arduino.h>

#ifdef ARDUINO_ARCH_STM32
CanController::CanController(): m_stm32CAN(CAN1, DEF) {}

void CanController::init()
{
    m_stm32CAN.begin();
    m_stm32CAN.setBaudRate(100000);
    // m_stm32CAN.enableLoopBack();
}

void CanController::send_can(t_can_frame frame) {
    CAN_message_t tx_msg;
    tx_msg.id = frame.id;
    tx_msg.len = frame.len;
    tx_msg.flags.remote = frame.rtr;
    // Serial.printf("data len: %d\n", data_len);
    memcpy(&tx_msg.buf, frame.buf, frame.len);
    if (m_stm32CAN.write(tx_msg)) {
        // Serial.println("can sent");
    };
}

bool CanController::receive_can(t_can_frame* frame) {
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
CanController::CanController() {}

void CanController::init(gpio_num_t rx, gpio_num_t tx) {
    // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
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
}

void CanController::send_can(uint32_t id, uint8_t *data, uint8_t data_len) {
    twai_message_t message;
    message.identifier = CAN_ID::LINE_RAW_SENSOR_DATA;
    message.data_length_code = data_len;
    Serial.printf("data len: %d\n", data_len);
    memcpy(&message.data, data, data_len);
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        printf("Message queued for transmission\n");
    } else {
        printf("Failed to queue message for transmission\n");
    }
}

bool CanController::receive_can(t_can_frame* frame) {
    //uint32_t alerts_triggered;
    //twai_read_alerts(&alerts_triggered, 0);
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_ERR_TIMEOUT) {
        Serial.println("no can frames");
        return false;
    }
    memcpy(frame->buf, &message.data, message.data_length_code);
    return true;
}
#endif

void CanController::send_rtr(CAN_ID msg_id) {
    t_can_frame frame {.id=msg_id, .rtr=true};
    send_can(frame);
}