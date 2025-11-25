#include "CanTxManager.h"
#include "Motor.h"
#include "main.h"
#include <stdint.h>


CAN_RxHeaderTypeDef rx_header;//id = 3 0x207
CAN_TxHeaderTypeDef tx_header_1 = {.StdId = 0x1FE,
                                 .ExtId = 0,
                                 .IDE = CAN_ID_STD,
                                 .RTR = CAN_RTR_DATA,
                                 .DLC = 8,
                                 .TransmitGlobalTime = DISABLE};

CAN_TxHeaderTypeDef tx_header_2 = {.StdId = 0x2FE,
                                 .ExtId = 0,
                                 .IDE = CAN_ID_STD,
                                 .RTR = CAN_RTR_DATA,
                                 .DLC = 8,
                                 .TransmitGlobalTime = DISABLE};

uint8_t tx_data_1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t tx_data_2[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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





void CanTxManager::init() {

}

void CanTxManager::SendCurrent1to4() {



    HAL_CAN_AddTxMessage(&hcan1, &tx_header_1, tx_data_1, NULL);
}


void CanTxManager::SendCurrent5to7() {

    HAL_CAN_AddTxMessage(&hcan1, &tx_header_2, tx_data_2, NULL);
}


void CanTxManager::SetCurrent(uint8_t motor_id, int16_t current) {
    // 电流值范围: -16384 ~ 16384
    if (motor_id >= 1 && motor_id <= 4) {
        // 电机ID 1-4 对应数据索引 0-7
        uint8_t index = (motor_id - 1) * 2;
        tx_data_1[index] = (uint8_t)(current >> 8);     // 高8位
        tx_data_1[index + 1] = (uint8_t)(current); // 低8位
    } else if (motor_id >= 5 && motor_id <= 7) {
        // 电机ID 5-7 对应数据索引 0-5
        uint8_t index = (motor_id - 5) * 2;
        tx_data_2[index] = (uint8_t)(current >> 8);     // 高8位
        tx_data_2[index + 1] = (uint8_t)(current); // 低8位
    }


}
