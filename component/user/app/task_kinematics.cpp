//
// Created by Henson on 2024-01-14.
//

#include "task_kinematics.h"
#include "drv_arm.h"

void kinematicsTask(void *argument){
    while(!g_arm.is_arm_offset_ok()) {
        osDelay(1);
    }
    osStatus_t stat;
    for(;;) {
        if(g_arm.communication.can.rx_custom_ctrl_data.is_using_custom_ctrl) {
            stat = osSemaphoreAcquire(g_arm.communication.can.can_custom_dev.rx.semaphore,5);
            if(osOK==stat){
                // 控制器模式的中间量
                g_arm.communication_custom_ctrl_data_to_encoder_direct_ctrl();
                g_arm.set_decomposition_for_custom_encoder_direct_ctrl();//kine的结果才是最终赋值给机械臂的结果 set_arm_pose只是中间量
            }else{
                g_arm.communication.can.rx_custom_ctrl_data.rcv_cnt=0;
            }
            //或者是osOK==stat && g_arm.communication.can.rx_custom_ctrl_data.is_using_custom_ctrl
        }else{
            g_arm.update_rotation_matrix();
            g_arm.solveIk();
            g_arm.solve_multiple_solutions();
        }
        g_arm.solveFk();
        osDelay(2);
    }
}

