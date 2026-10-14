//
// Created by Siwei Wang on 2026/10/14.
//

#ifndef ROBOMASTER27ECTRAINING4TH_MOTOR_HPP
#define ROBOMASTER27ECTRAINING4TH_MOTOR_HPP
#include <cstdint>
#include "PID.hpp"
extern uint8_t tx_data[8];

class Motor
{
public:
    explicit Motor(const float ratio);

    void canRxMsgCallback(const uint8_t rx_data[8]);

    float angle() const;
    int16_t speed_rpm() const;
    float currentAmps() const;
    int16_t temperatureC() const;

    void setTxCurrent(float amperes, uint8_t motor_id);

    uint8_t* getTxData();

    // PID spid, ppid;
    // float target_angle_, fdb_angle_;
    // float target_speed_, fdb_speed_;
    // float feedforward_speed_, feedforward_intensity_;
    // typedef enum ControlMethod{
    //     TARGET_POSITION,
    //     TARGET_SPEED,
    //     TARGET_TORQUE,
    // } ControlMethod_e;
    // void handle();
    // void setSpeed(float target_speed, float feedforward_intensity);

private:
    const float ratio_;

    int16_t ecd_angle_ = 0;
    int16_t speed_rpm_ = 0;
    float currentA_ = 0;
    int16_t tempC_ = 0;

    float angle_ = 0;
    int16_t last_ecd_ = 0;
    // bool received_ = false;

    uint8_t tx_data_[8] = {};

    void normalizeDataReceived();

    static constexpr uint16_t kEncoderRange = 8192;
};

#endif //ROBOMASTER27ECTRAINING4TH_MOTOR_HPP
