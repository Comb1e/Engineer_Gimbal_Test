//
// Created by Henson on 2024-01-14.
//

#include "task_IMU.h"
#include "drv_arm.h"
#include "bsp_led.h"

void armRollIMUTask(void *argument) {
    osStatus_t status;
    while(!g_arm.is_init){
        osDelay(1);
    }
    HAL_UART_RegisterRxEventCallback(g_arm.actuator.arm_roll_IMU.huart, armRollIMUCallBack);
    HAL_UART_RegisterCallback(g_arm.actuator.arm_roll_IMU.huart, HAL_UART_ERROR_CB_ID, armRollIMUUnlinkCallBack);
    g_arm.actuator.arm_roll_IMU.instruction_set(dir_cmd_z_90);
    g_arm.actuator.arm_roll_IMU.instruction_set(mode_cmd_9_axis);
    for(;;) {
        g_arm.actuator.arm_roll_IMU.start_receive_dma();
        status = osSemaphoreAcquire(g_arm.actuator.arm_roll_IMU.rx_semaphore, 25);
        if (osOK == status) {
            g_arm.actuator.arm_roll_IMU.set_imu_connect();
            if (g_arm.actuator.arm_roll_IMU.is_data_legal()) {
                g_arm.actuator.arm_roll_IMU.update_imu();
                g_arm.actuator.arm_roll_IMU.rcv_cnt++;
            } else {
                g_arm.actuator.arm_roll_IMU.error_data_cnt++;
            }
            set_red_off();
        }else{
            set_red_on();
            g_arm.actuator.arm_roll_IMU.lost_cnt++;
            g_arm.actuator.arm_roll_IMU.set_imu_lost();
        }
    }
}

void armRollIMUCallBack(UART_HandleTypeDef *huart, uint16_t Size){
    osSemaphoreRelease(g_arm.actuator.arm_roll_IMU.rx_semaphore);
}

void armRollIMUUnlinkCallBack(UART_HandleTypeDef *huart){
    ;
}