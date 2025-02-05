//
// Created by Henson on 2024-03-22.
//

#include "task_auto.h"
#include "drv_arm.h"
#include "drv_buzzer.h"

void autoCtrlTask(void *argument)
{
    osStatus_t stat;
    for(;;)
    {
        g_arm.framework.is_all_motors_safe = g_arm.framework.check_all_motors_safe();
        g_arm.actuator.is_all_motors_safe = g_arm.actuator.check_all_motors_safe();
        g_arm.is_all_peripheral_connect = g_arm.framework.is_all_motors_safe && g_arm.actuator.is_all_motors_safe && !g_arm.actuator.arm_roll_IMU.is_imu_lost();
        g_arm.check_motors_overheat();

        /*if(g_arm.is_all_peripheral_connect)
        {
            buzzer_off();
        }
        else
        {
            buzzer_on();
        }*/
        g_arm.is_none_of_peripheral_connect =
                   g_arm.framework.extend_l.is_motor_lost() && g_arm.framework.extend_r.is_motor_lost()
                && g_arm.framework.uplift_l.is_motor_lost() && g_arm.framework.uplift_r.is_motor_lost()
                && g_arm.framework.slide.is_motor_lost() && g_arm.framework.arm_yaw.is_motor_lost()
                && g_arm.framework.arm_pitch.is_motor_lost()
                && g_arm.actuator.arm_roll.is_motor_lost() && g_arm.actuator.pitch_roll_b.is_motor_lost()
                && g_arm.actuator.pitch_roll_f.is_motor_lost();
#if AUTO_REVIVE
        if(g_arm.is_enable && g_arm.is_none_of_peripheral_connect)
        {// 这个其实可以用板子复位来做  NO!
            g_arm.set_arm_reset();
        }
#endif

        osDelay(1);
    }
}