//
// Created by Henson on 2023-10-22.
//

#ifndef __FREERTOS_INC_H
#define __FREERTOS_INC_H

#include "FreeRTOS.h"
#include "cmsis_os.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern osSemaphoreId sem_can1_tx;
extern osSemaphoreId sem_can2_tx;
extern osSemaphoreId sem_joint_attitude_rx;
extern osSemaphoreId sem_joint_forearm_rx;
extern osSemaphoreId sem_joint_upper_arm_rx;
extern osSemaphoreId_t dr16UpdateBinarySemHandle;
extern osSemaphoreId_t CommCanBinarySemHandle;
extern osSemaphoreId_t upliftLeftBinarySemHandle;
extern osSemaphoreId_t upliftRightBinarySemHandle;
extern osSemaphoreId_t extendLeftBinarySemHandle;
extern osSemaphoreId_t extendRightBinarySemHandle;
extern osSemaphoreId_t slideBinarySemHandle;
extern osSemaphoreId_t armYawBinarySemHandle;
extern osSemaphoreId_t yawExtendBinarySemHandle;
extern osSemaphoreId_t armRollBinarySemHandle;
extern osSemaphoreId_t pitchRollUpBinarySemHandle;
extern osSemaphoreId_t pitchRollDownBinarySemHandle;
extern osSemaphoreId_t cameraYawBinarySemHandle;
extern osSemaphoreId_t armRollImuBinarySemHandle;
extern osSemaphoreId_t can1_tx_cnt_semHandle;
extern osSemaphoreId_t can2_tx_cnt_semHandle;
extern osSemaphoreId_t armYawDMBinarySemHandle;
extern osSemaphoreId_t armPitchDMBinarySemHandle;
extern osSemaphoreId_t USBUpdateBinarySemHandle;

extern osMessageQueueId_t can1SendQueueHandle;
extern osMessageQueueId_t can2SendQueueHandle;

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__FREERTOS_INC_H
