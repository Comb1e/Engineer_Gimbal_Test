//
// Created by Henson on 2024-01-14.
//

#include "task_comm.h"
#include "interface_can.h"
#include "drv_arm.h"
#include "drv_key.h"
#include "bsp_led.h"

void communicationTask(void *argument)
{
    communication.Init();
    communication.PTR_Init(&comm_can,&comm_usb,&g_arm,&g_rc);
    communication.arm->Arm_init();
    osStatus_t stat;
    osSemaphoreAcquire(communication.can->can_device.rx.semaphore,osWaitForever);// all the first
    communication.arm->set_arm_reset();
    set_blue_on();
    for(;;)
    {
        stat = osSemaphoreAcquire(communication.can->can_device.rx.semaphore,6);
        if(osOK == stat)
        {
            // 正常模式的中间量
            communication.Update_Data();
            set_blue_on();
        }
        else
        {
            set_blue_off();
        }
        osDelay(4);
    }
}