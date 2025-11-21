#include "CanTxManager.h"
#include "Motor.h"
#include "main.h"
#include <stdint.h>


void CanTxManager::init() {

}

void CanTxManager::SendCurrent1to4() {



    HAL_CAN_AddTxMessage(&hcan1, &tx_header_1, tx_data, NULL);
}


void CanTxManager::SendCurrent5to7() {

    HAL_CAN_AddTxMessage(&hcan1, &tx_header_2, tx_data, NULL);
}


void CanTxManager::SetCurrent=(uint8_t motor_id, int16_t Current) {



}
