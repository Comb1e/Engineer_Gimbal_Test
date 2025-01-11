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

    osStatus_t status = osSemaphoreAcquire(USBUpdateBinarySemHandle,15);
    while(status != osOK)
    {
        status = osSemaphoreAcquire(USBUpdateBinarySemHandle,15);
    }//接收到usb第一次消息开始下一步

    communication.arm->Arm_init();
    communication.arm->set_arm_reset();
    set_blue_on();
    for(;;)
    {
        communication.Update_Lost_Flag();
        if(communication.Check_Lost())
        {
            set_blue_off();
        }
        else
        {
            communication.arm->Update_Arm_Current_Position();
            communication.usb->Receive_Data();
            communication.Update_Data();
            communication.Send_MSG();
            set_blue_on();
        }
        osDelay(4);
    }
}