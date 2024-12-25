//
// Created by Henson on 2024-03-27.
//

#ifndef __DRV_BUZZER_H
#define __DRV_BUZZER_H

/*---------------------------- C++ Scope ---------------------------*/

#include "tim.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void buzzer_init();
void buzzer_on();
void buzzer_off();

#ifdef __cplusplus
}
#endif

#endif //__DRV_BUZZER_H
