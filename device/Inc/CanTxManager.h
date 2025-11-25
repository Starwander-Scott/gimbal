//
// Created by cycsjtuer on 2025/11/11.
//

#ifndef FINAL_CANTXMANAGER_H
#define FINAL_CANTXMANAGER_H

#include <can.h>

extern CAN_RxHeaderTypeDef rx_header;
// extern CAN_TxHeaderTypeDef tx_header_1;
extern CAN_TxHeaderTypeDef tx_header_2;
// extern uint8_t tx_data_1[8];
extern uint8_t tx_data_2[8];
extern uint8_t rx_data[8];
extern uint32_t can_tx_mail_box_;
extern CAN_FilterTypeDef filter_config;




#ifdef __cplusplus

class CanTxManager {
private:


public:
    void init();
    static void SetCurrent(uint8_t motor_id, int16_t current);
    static void SendCurrent1to4();
    static void SendCurrent5to7();
};


#endif //__cplusplus

#endif //FINAL_CANTXMANAGER_H