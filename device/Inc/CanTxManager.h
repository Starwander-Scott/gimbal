//
// Created by cycsjtuer on 2025/11/11.
//

#ifndef FINAL_CANTXMANAGER_H
#define FINAL_CANTXMANAGER_H

#include <can.h>

CAN_RxHeaderTypeDef rx_header;
CAN_TxHeaderTypeDef tx_header_1 = {.StdId = 0x1FE,
                                 .ExtId = 0,
                                 .IDE = CAN_ID_STD,
                                 .RTR = CAN_RTR_DATA,
                                 .DLC = 8,
                                 .TransmitGlobalTime = DISABLE};

CAN_TxHeaderTypeDef tx_header_2 = {.StdId = 0x2FF,
                                 .ExtId = 0,
                                 .IDE = CAN_ID_STD,
                                 .RTR = CAN_RTR_DATA,
                                 .DLC = 8,
                                 .TransmitGlobalTime = DISABLE};

uint8_t tx_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t rx_data[8];
uint32_t can_tx_mail_box_;

CAN_FilterTypeDef filter_config = {
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = ENABLE,
};



class CanTxManager {
private:


public:
    void init();
    static void SetVoltage(uint8_t motor_id, int16_t voltage);
    static void SendVoltage1to4();
    static void SendVoltage5to7();
};




#endif //FINAL_CANTXMANAGER_H