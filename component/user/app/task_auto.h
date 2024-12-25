//
// Created by Henson on 2024-03-22.
//

#ifndef __TASK_AUTO_H
#define __TASK_AUTO_H

#include "freertos_inc.h"
#include "GlobalCfg.h"
/*---------------------------- C++ Scope ---------------------------*/


/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void autoCtrlTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif //__TASK_AUTO_H
