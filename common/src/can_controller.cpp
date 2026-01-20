#include "can_controller.h"
#include <Arduino.h>

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