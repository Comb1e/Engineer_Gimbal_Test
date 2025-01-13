//
// Created by Henson on 2023-10-22.
//

#ifndef __GLOBALCFG_H
#define __GLOBALCFG_H

#define ARM_ROLL_IMU        huart1  //460800
#define RC_UART             huart3

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

#define GIMBAL_TO_CHASSIS_RX_CAN2_STDID (0X301)
#define GIMBAL_TO_CHASSIS_TX_CAN2_STDID (0X303)

// ------------------- IIC ------------------- //

#define IIC_GPIO            GPIOE
#define IIC_SCL_PIN         GPIO_PIN_9
#define IIC_SDA_PIN         GPIO_PIN_11

#define AUTO_REVIVE         1
#define RC_ENABLE           1
#define DJI_MOTOR_SERVICE   1

/*----------TEST----------*/
#define GC_COMMUNICATION_TEST 0

/*---------------------------- C Scope --------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__GLOBALCFG_H
