#include "can_controller.h"
#include <Arduino.h>

CanController::CanController(): m_stm32CAN(STM32_CAN(CAN1, DEF)), m_tx_msg{} {
}

void CanController::init()
{
    m_stm32CAN = STM32_CAN(CAN1, DEF);
    m_stm32CAN.begin();
    m_stm32CAN.setBaudRate(1000000);
}

void CanController::send_can(uint32_t id, uint8_t *data, uint8_t data_len) {
    m_tx_msg.id = (CAN_ID::LINE_RAW_SENSOR_DATA);
    m_tx_msg.len = data_len;
    memcpy(m_tx_msg.buf, &data, data_len);
    m_stm32CAN.write(m_tx_msg);
}

void CanController::handle_can() {
    t_line_sensor_data data = {  };
    handle_struct(data);
}

void CanController::send_struct(t_line_sensor_raw_data data) {
    send_can(CAN_ID::LINE_RAW_SENSOR_DATA, (uint8_t*) &data, sizeof(data));
}

void CanController::send_struct(t_line_sensor_data data) {
    send_can(CAN_ID::LINE_SENSOR_DATA, (uint8_t*) &data, sizeof(data));
}
