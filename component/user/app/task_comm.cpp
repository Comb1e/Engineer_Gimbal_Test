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
    while(communication.rc->is_lost)
    {
        communication.usb->Update_RX_Data();
        osDelay(1);
    }
    communication.arm->set_arm_reset();
    set_blue_on();
    osDelay(100);
#if ARM_CAN_OPEN
    while(!communication.arm->is_arm_reset_all_ok())
    {
        osDelay(1);
    }
#endif
    for(;;)
    {
        communication.arm->Update_Arm_Current_Position();
        communication.usb->Update_RX_Data();
        communication.Update_TX_Data();
        communication.Update_USB_Lost_Flag();
        if(communication.Check_USB_Lost_Flag() || communication.rc->is_lost)
        {
            set_blue_off();
            communication.Set_Chassis_Control_RC();
            communication.usb->Transmit_Data();
        }
        else
        {
            communication.Update_Arm_Control();
            communication.Set_Chassis_Control_USB();
            communication.Send_MSG();
            set_blue_on();
        }
        osDelay(2);
    }
}