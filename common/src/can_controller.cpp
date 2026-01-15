#include "can_controller.h"
#include <Arduino.h>


CanController::CanController(): m_stm32CAN(STM32_CAN(CAN1, DEF)), m_tx_msg {}, m_rx_msg {} {}

void CanController::init()
{
    m_stm32CAN = STM32_CAN(CAN1, DEF);
    Serial.println("1");
    delay(1000);
    m_stm32CAN.begin();
    Serial.println("2");
    delay(1000);
    m_stm32CAN.enableLoopBack();
    Serial.println("3");
    delay(1000);
    m_stm32CAN.setBaudRate(1000000);
    Serial.println("4");
    delay(3000);
    Serial.println("tjrs pas crashahahah");
}

void CanController::send_can(uint32_t id, uint8_t *data, uint8_t data_len) {
    m_tx_msg.id = CAN_ID::LINE_RAW_SENSOR_DATA;
    m_tx_msg.len = data_len;
    Serial.printf("data len: %d\n", data_len);
    delay(1000);
    memcpy(&m_tx_msg.buf, data, data_len);
    Serial.println("done memcpy");
    delay(1000);
    if (m_stm32CAN.write(m_tx_msg)) {
        delay(1000);
        Serial.println("can sent");
    };
}

void CanController::receive_can(t_can_frame* frame) {
    if (!m_stm32CAN.read(m_rx_msg)){
        Serial.println("no can frames");
        return;
    }

    frame->id = m_rx_msg.id;
    frame->len = m_rx_msg.len;
    memcpy(frame->buf, &m_rx_msg.buf, m_rx_msg.len);
}

void CanController::handle_can() {
    t_can_frame frame;
    receive_can(&frame);
    uint32_t can_msg_id = m_rx_msg.id;
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