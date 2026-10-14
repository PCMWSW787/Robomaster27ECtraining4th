//
// Created by Siwei Wang on 2026/10/14.
//

#include "callback.hpp"


#include "can.h"
#include "can_user.h"
#include "Motor.hpp"
#include "tim.h"

PID ppid(0.0f, 0.0f, 0.0f, 0.0f, 8192.0f, 0.0f);
PID spid(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f);

Motor motor1(36.0f, spid, ppid, 4, ControlMethod::TARGET_POSITION, MotorType::M2006);

uint8_t tx_data[8] = {};

float test_target_position = 0.0f, test_target_speed = 0.0f, test_target_intensity = 0.0f;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t data[8];
    if (hcan != &hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) return;
    if (rx_header.IDE == CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA && rx_header.DLC == 8)
    {
        if (rx_header.StdId == motor1.getFeedbackId() )
            motor1.canRxMsgCallback(data);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim6) return;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        motor1.setTarget(test_target_position, test_target_speed, test_target_intensity);
        motor1.handle();
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mailbox);
    }
}