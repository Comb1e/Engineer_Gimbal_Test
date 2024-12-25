//
// Created by Henson on 2024-01-14.
//

#include "task_motor.h"
#include "drv_arm.h"

void motorTask(void *argument) {
    for(;;) {
        while(!g_arm.is_init){
            osDelay(1);
        }
        if(!g_arm.actuator.check_all_motors_safe()) {//不safe
            __NOP();
        }
        if(!g_arm.framework.check_all_motors_safe()) {//不safe
            __NOP();
        }
        osDelay(3);
    }
}

#if DJI_MOTOR_SERVICE
void djiMotorServiceTask(void *argument) {
    CAN can;
    uint32_t tick = osKernelGetTickCount();
    while(!g_arm.is_all_peripheral_connect) {
        osDelay(1);
    }
    for(;;){
        for(uint32_t i=0;i<6;i++) {
            if(DJI_Motor::can_tx_list[i].is_enable){
                can.hcan = DJI_Motor::can_tx_list[i].hcan;
                can.tx.stdid = DJI_Motor::can_tx_list[i].stdid;
                can.tx.len = DJI_Motor::can_tx_list[i].len;
                can.tx.buff = DJI_Motor::can_tx_list[i].buff;
                can.send_can_msg();
            }
        }
        tick+=3;
        osDelayUntil(tick);
    }
}
#endif
