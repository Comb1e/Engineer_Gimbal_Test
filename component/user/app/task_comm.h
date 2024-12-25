//
// Created by Henson on 2024-01-14.
//

#ifndef __TASK_COMM_H
#define __TASK_COMM_H

#include "drv_comm.h"
#include "freertos_inc.h"
#include "GlobalCfg.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void communicationTask(void *argument);

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/







#endif //__TASK_COMM_H
