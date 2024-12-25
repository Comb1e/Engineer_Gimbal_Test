//
// Created by Henson on 2024-01-14.
//

#ifndef __TASK_CTRL_H
#define __TASK_CTRL_H

#include "freertos_inc.h"
#include "GlobalCfg.h"
/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void extendCtrlTask(void *argument);
void slideCtrlTask(void *argument);
void upliftCtrlTask(void *argument);
void armYawCtrlTask(void *argument);
void armPitchCtrlTask(void *argument);
void armRollCtrlTask(void *argument);
void yX2CtrlTask(void *argument);


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__TASK_CTRL_H
