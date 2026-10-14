//
// Created by Siwei Wang on 2026/10/14.
//

#ifndef ROBOMASTER27ECTRAINING4TH_CAN_USER_H
#define ROBOMASTER27ECTRAINING4TH_CAN_USER_H
#include "stm32f4xx_hal.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern CAN_TxHeaderTypeDef tx_header1;
extern CAN_TxHeaderTypeDef tx_header2;

extern uint32_t can_tx_mailbox;
extern CAN_FilterTypeDef can_filter_config;

void canRxMsgCallback(CAN_HandleTypeDef *hcan);
#endif //ROBOMASTER27ECTRAINING4TH_CAN_USER_H
