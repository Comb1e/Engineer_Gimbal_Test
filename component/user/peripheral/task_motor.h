//
// Created by Henson on 2024-01-14.
//

#ifndef __TASK_MOTOR_H
#define __TASK_MOTOR_H

#include "freertos_inc.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void motorTask(void *argument);
void djiMotorServiceTask(void *argument);

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__TASK_MOTOR_H
