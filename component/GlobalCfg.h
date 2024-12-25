//
// Created by Henson on 2023-10-22.
//

#ifndef __GLOBALCFG_H
#define __GLOBALCFG_H

#define ARM_ROLL_IMU        huart1  //460800

// --------------------- CAN 500Hz--------------------- //
// ------------------- CAN1 ------------------- //
//包(1+2)x2
#define ARM_YAW_CAN         hcan1   //0x02
#define ARM_PITCH_CAN       hcan1   //0x04
//包3+1
#define ARM_ROLL_CAN        hcan1   //ID1
#define PITCH_ROLL_U_CAN    hcan1   //ID2
#define PITCH_ROLL_D_CAN    hcan1   //ID3

// ------------------- CAN2 ------------------- //
//包4+1
#define UPLIFT_L_CAN        hcan2   //ID1
#define UPLIFT_R_CAN        hcan2   //ID2
#define EXTEND_L_CAN        hcan2   //ID3
#define EXTEND_R_CAN        hcan2   //ID4
//包1+1
#define SLIDE_CAN           hcan2   //ID5

//包5
#define COMM_DATA_CAN                   hcan2

#define COMM_CAN_BASIC_RX_STDID         0x301
#define COMM_CAN_MORE_RX_STDID          0x302

#define COMM_CAN_BASIC_TX_STDID         0x303
#define COMM_CAN_MORE_TX_STDID          0x304

#define COMM_CAN_CUSTOM_RX_STDID        0x305
#define COMM_CAN_CUSTOM_TX_STDID        0x306

// ------------------- IIC ------------------- //

#define IIC_GPIO            GPIOE
#define IIC_SCL_PIN         GPIO_PIN_9
#define IIC_SDA_PIN         GPIO_PIN_11

#define AUTO_REVIVE         1
#define RC_ENABLE           0
#define DJI_MOTOR_SERVICE   1

/*---------------------------- C Scope --------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__GLOBALCFG_H
