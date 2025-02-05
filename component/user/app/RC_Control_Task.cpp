//
// Created by CYK on 2025/1/18.
//

#include "RC_Control_Task.h"

#include "cmsis_os2.h"
#include "drv_comm.h"

void RC_Control_Task(void *argument)
{
    for(;;)
    {
        if(communication.rc_control_flag)
        {
            communication.Update_RC_Control();
            communication.can->can_device.send_can_msg();
        }
        osDelay(1);
    }
}
