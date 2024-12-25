//
// Created by Henson on 2024-01-14.
//

#include "task_comm.h"
#include "interface_can.h"
#include "drv_arm.h"
#include "drv_key.h"
#include "bsp_led.h"

void communicationTask(void *argument) {
    g_arm.Arm_init();
    osStatus_t stat;
    osSemaphoreAcquire(g_arm.communication.can.can_basic_dev.rx.semaphore,osWaitForever); // all the first
    g_arm.set_communication_basic_data_to_rx_data();
    g_arm.set_communication_to_arm_pose();
    g_arm.set_arm_reset();
    set_blue_on();
    for(;;) {
        stat = osSemaphoreAcquire(g_arm.communication.can.can_basic_dev.rx.semaphore,6);//这里只是检查basic，没有检查其它包
        if(osOK == stat) {
            // 正常模式的中间量
            g_arm.set_communication_basic_data_to_rx_data();
            g_arm.set_communication_to_arm_pose();
            set_blue_on();
        }else{
            set_blue_off();
        }
        if(g_arm.is_fk_data_valid) {
            g_arm.set_FK_data_to_communication();
            g_arm.send_FK_data_to_communication();
        }
        osDelay(4);
    }
}