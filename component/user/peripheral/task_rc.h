//
// Created by Henson on 2024-01-14.
//

#ifndef __TASK_RC_H
#define __TASK_RC_H

#include "drv_dr16.h"
#include "GlobalCfg.h"
#include "freertos_inc.h"
/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void rcReceiveTask(void *argument);
void dr16_update_callback(UART_HandleTypeDef *huart, uint16_t Size);
void dr16_unlink_restart(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__TASK_RC_H
