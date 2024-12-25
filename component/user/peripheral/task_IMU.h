//
// Created by Henson on 2024-01-14.
//

#ifndef __TASK_IMU_H
#define __TASK_IMU_H

#include "drv_imu.h"
#include "freertos_inc.h"
/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void armRollIMUTask(void *argument);
void armRollIMUCallBack(UART_HandleTypeDef *huart, uint16_t Size);
void armRollIMUUnlinkCallBack(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__TASK_IMU_H
