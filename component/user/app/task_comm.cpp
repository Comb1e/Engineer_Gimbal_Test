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
    communication.PTR_Init(&comm_can,&comm_usb,&g_arm,&g_rc);
    communication.Init();
    communication.arm->Arm_init();
    communication.usb->Start_Receive_Data();
    communication.arm->set_arm_reset();
    set_blue_on();
    osDelay(6000);
    for(;;)
    {
        communication.arm->Update_Arm_Current_Position();
        communication.usb->Update_RX_Data();
        communication.Update_TX_Data();
        communication.Update_USB_Lost_Flag();
        if(communication.Check_USB_Lost_Flag())
        {
            set_blue_off();
            communication.Update_Chassis_Control_RC();
            debug0=5;
        }
        else
        {
            communication.Update_Arm_Control();
            set_blue_on();
        }
        communication.Send_MSG();
        osDelay(2);
    }
}