#include "can_controller.h"
#include <Arduino.h>

#ifdef ARDUINO_ARCH_STM32
CanController::CanController(): m_stm32CAN(STM32_CAN(CAN1, DEF)), m_tx_msg {}, m_rx_msg {} {}

void CanController::init()
{
    m_stm32CAN = STM32_CAN(CAN1, DEF);
    m_stm32CAN.begin(false);
    m_stm32CAN.setBaudRate(100000);
    // m_stm32CAN.enableLoopBack();
}

void CanController::send_can(uint32_t id, uint8_t *data, uint8_t data_len) {
    m_tx_msg.id = CAN_ID::LINE_RAW_SENSOR_DATA;
    m_tx_msg.len = data_len;
    Serial.printf("data len: %d\n", data_len);
    memcpy(&m_tx_msg.buf, data, data_len);
    if (m_stm32CAN.write(m_tx_msg)) {
        Serial.println("can sent");
    };
}

bool CanController::receive_can(t_can_frame* frame) {
    if (!m_stm32CAN.read(m_rx_msg)){
        Serial.println("no can frames");
        return false;
    }

    frame->id = m_rx_msg.id;
    frame->len = m_rx_msg.len;
    memcpy(frame->buf, &m_rx_msg.buf, m_rx_msg.len);
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

void CanController::handle_can() {
    t_can_frame frame;
    receive_can(&frame);
    uint32_t can_msg_id = frame.id;
    switch (can_msg_id) {
    case CAN_ID::LINE_RAW_SENSOR_DATA: {
        t_line_sensor_raw_data data;
        memcpy(&data, frame.buf, sizeof(data));
        handle_struct(data);
        break;
    }
    case CAN_ID::LINE_SENSOR_DATA: {
        t_line_sensor_data data;
        memcpy(&data, frame.buf, sizeof(data));
        handle_struct(data);
        break;
    } 
    }
}

void CanController::send_struct(t_line_sensor_raw_data data) {
    send_can(CAN_ID::LINE_RAW_SENSOR_DATA, (uint8_t*) &data, sizeof(data));
}

void CanController::send_struct(t_line_sensor_data data) {
    send_can(CAN_ID::LINE_SENSOR_DATA, (uint8_t*) &data, sizeof(data));
}