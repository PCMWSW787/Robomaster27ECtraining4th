//
// Created by Siwei Wang on 2026/10/14.
//

#include "motor.hpp"
#include "stm32f4xx_hal.h"
extern CAN_TxHeaderTypeDef tx_header;
extern CAN_TxHeaderTypeDef tx_header1;
extern CAN_TxHeaderTypeDef tx_header2;

Motor::Motor(const float ratio, PID spid, PID ppid, uint8_t motor_id)
    :ratio_(ratio),
    spid_(spid),
    ppid_(ppid),
    motor_id_(motor_id)
{}

float Motor::angle() const
{
    return angle_;
}
int16_t Motor::temperatureC() const
{
    return tempC_;
}
float Motor::currentAmps() const
{
    return currentA_;
}

int16_t Motor::speed_rpm() const
{
    return speed_rpm_;
}

uint32_t Motor::getFeedbackId()
{
    return 0x200 + static_cast<uint32_t>(motor_id_);
}

void Motor::normalizeDataReceived()
{
    float angle_deg = static_cast<float>(ecd_angle_) * 360.0f / 8192.0f;
    float last_angle_deg = static_cast<float>(last_ecd_) * 360.0f / 8192.0f;
    float delta_angle_deg = angle_deg - last_angle_deg;
    if (delta_angle_deg > 180.0f) delta_angle_deg -= 360.0f;
    else if (delta_angle_deg < -180.0f) delta_angle_deg += 360.0f;
    angle_ += delta_angle_deg / ratio_;
    last_ecd_ = ecd_angle_;
}

void Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    ecd_angle_ = static_cast<int16_t>((static_cast<uint16_t>(rx_data[0]) << 8) | static_cast<uint16_t>(rx_data[1]));
    speed_rpm_ = static_cast<int16_t>((static_cast<uint16_t>(rx_data[2]) << 8) | static_cast<uint16_t>(rx_data[3])) / ratio_;
    // int16_t rx_raw_current = static_cast<int16_t>(static_cast<uint16_t>(rx_data[4]) << 8 | (static_cast<uint16_t>(rx_data[5])));
    // currentA_ = static_cast<float>(rx_raw_current) * 10.0f / 10000.0f;
    // tempC_ = static_cast<int16_t>(rx_data[6]);
    normalizeDataReceived();
}

void Motor::setTxCurrent(float amperes, uint8_t motor_id)
{
    if (amperes > 10.0f) amperes = 10.0f;
    else if (amperes < -10.0f) amperes = -10.0f;
    int16_t tx_raw_current = static_cast<int16_t>(amperes * 10000.0f / 10.0f);
    switch (motor_id)
    {
    case 1:
        tx_header = tx_header1;
        tx_data[0] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) >> 8 & 0xFF);
        tx_data[1] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) & 0xFF);
        break;
    case 2:
        tx_header = tx_header1;
        tx_data[2] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) >> 8 & 0xFF);
        tx_data[3] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) & 0xFF);
        break;
    case 3:
        tx_header = tx_header1;
        tx_data[4] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) >> 8 & 0xFF);
        tx_data[5] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) & 0xFF);
        break;
    case 4:
        tx_header = tx_header1;
        tx_data[6] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) >> 8 & 0xFF);
        tx_data[7] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) & 0xFF);
        break;
    case 5:
        tx_header = tx_header2;
        tx_data[0] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) >> 8 & 0xFF);
        tx_data[1] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) & 0xFF);
        break;
    case 6:
        tx_header = tx_header2;
        tx_data[2] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) >> 8 & 0xFF);
        tx_data[3] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) & 0xFF);
        break;
    case 7:
        tx_header = tx_header2;
        tx_data[4] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) >> 8 & 0xFF);
        tx_data[5] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) & 0xFF);
        break;
    case 8:
        tx_header = tx_header2;
        tx_data[6] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) >> 8 & 0xFF);
        tx_data[7] = static_cast<uint8_t>(static_cast<uint16_t>(tx_raw_current) & 0xFF);
        break;
    default:
        break;
    }
}

void Motor::handle()
{
    fdb_angle_ = angle_;
    fdb_speed_ = speed_rpm_;
    target_speed_ = ppid_.calc(target_angle_,angle_);
    float out_current = spid_.calc(target_speed_,speed_rpm_);
    setTxCurrent(out_current, motor_id_);
}

uint8_t* Motor::getTxData()
{
    return tx_data_;
}