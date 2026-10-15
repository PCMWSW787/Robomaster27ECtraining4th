//
// Created by Siwei Wang on 2026/10/14.
//

#ifndef ROBOMASTER27ECTRAINING4TH_MOTOR_HPP
#define ROBOMASTER27ECTRAINING4TH_MOTOR_HPP
#include <cstdint>
#include "PID.hpp"
extern uint8_t tx_data[8];

enum class ControlMethod
{
    TARGET_POSITION,
    TARGET_SPEED,
    TARGET_TORQUE,
};


enum class MotorType
{
    M2006 = 10000,
    M3508 = 16384
};

class Motor
{
public:
    //construct the class
    explicit Motor(const float ratio, PID spid, PID ppid, uint8_t motor_id, ControlMethod control_method, MotorType motor_type);

    //solve the can message
    void canRxMsgCallback(const uint8_t rx_data[8]);

    //apis
    float angle() const;
    int16_t speed_rpm() const;
    float currentAmps() const;
    int16_t temperatureC() const;

    //output
    void setTxCurrent(float amperes, uint8_t motor_id);

    //send can message
    uint8_t* getTxData();

    //get feedback id
    uint32_t getFeedbackId();

    //control module
    void handle();
    void setTarget(float target_position_deg, float target_speed_rpm, float target_intensity_amps);
    void setSpeed(float target_speed_rpm, float feedforward_intensity_amps);
    void setPosition(float target_position_deg, float feedforward_speed_rpm, float feedforward_intensity);
    void setFeedforward(float feedforward_speed_rpm, float feedforward_intensity_amps);
    void setIntensity(float intensity);

private:
    const float ratio_;
    MotorType motor_type_;

    int16_t ecd_angle_ = 0, last_ecd_angle_ = 0;
    int16_t fdb_speed_rpm_ = 0;
    float fdb_current_amps = 0.0f;
    float fdb_angle_deg_ = 0.0f;
    float integration_angle_deg_ = 0.0f;
    int16_t tempC_ = 0;

    // bool received_ = false;

    uint8_t tx_data_[8] = {};

    void normalizeDataReceived();

    PID spid_, ppid_;
    float target_position_deg_ = 0.0f, target_speed_rpm_ = 0.0f, target_intensity_amps_ = 0.0f;
    float feedforward_speed_rpm_ = 0.0f, feedforward_intensity_amps_ = 0.0f;
    float output_current_amps_ = 0.0f;
    ControlMethod control_method_;

    uint8_t motor_id_ = 0;

    static constexpr uint16_t kEncoderRange = 8192;
};

#endif //ROBOMASTER27ECTRAINING4TH_MOTOR_HPP
