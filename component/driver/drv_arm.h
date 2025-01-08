//
// Created by Henson on 2024-03-04.
//

#ifndef __DRV_ARM_H
#define __DRV_ARM_H

/*---------------------------- C++ Scope ---------------------------*/
#include "drv_actuator.h"
#include "drv_framework.h"
#include "robotics_math.h"
#include "drv_dof.h"
#include "user_lib.h"
#include "GlobalCfg.h"

const float RAD_TO_DEG = 57.295777754771045f;

#define X1_CNT_MAX  9
#define Y_CNT_MAX   5
#define X2_CNT_MAX  9

#define ARM_X_MM_OFFSET     (-200)
#define ARM_Y_MM_OFFSET     400
#define ARM_Z_MM_OFFSET     150

typedef struct
{
    float uplift_joint;
    float extend_joint;
    float slide_joint;
    float arm_yaw_joint;
    float arm_pitch_joint;
    float arm_roll_joint;
    float pitch_joint;
    float roll_joint;
}arm_joint_t;

typedef struct
{
    float dist;
    bool is_good;
    Eigen::Vector3f deg;
}MulSolu_search_t;

class Arm
{
public:
    explicit Arm();

    Framework framework;
    Actuator actuator;
    //root
    dof6_t offset = {0};
    dof6_t get = {0};
    dof6_t set = {420,70.0f,50.0f,0.0f,0.0f,0.0f};
    dof6_t min = {0};
    dof6_t max = {0};
    float set_arm_yaw_deg = 0.0f;
    float set_arm_pitch_deg = 0.0f;
    //flag
    bool is_init = false;
    bool is_enable = false;
    bool is_all_peripheral_connect = false;
    bool is_none_of_peripheral_connect = true;
    bool is_motors_overheat = false;
    uint32_t arm_reset_num = 0;

    arm_joint_t set_joint;
    arm_joint_t current_joint;

    void Arm_init();
    void Update_Data();

    //get
    bool is_arm_offset_ok();
    bool is_arm_at_x_lower_limit();
    bool is_arm_at_y_lower_limit();
    bool is_arm_at_x_higher_limit();
    bool is_arm_at_y_higher_limit();

    //reset
    void set_arm_reset();
    void set_actuator_single_reset();
    void set_framework_single_reset();
    void set_reset_pid();

    //run
    void set_arm_move();
    void set_arm_pose(dof6_e dofE, float set_mm_deg);//接口
    void set_arm_arm_yaw_deg(float set_deg);
    void set_arm_arm_pitch_deg(float set_deg);
    bool is_arm_at_initial_pos();
    bool is_arm_reset_all_ok();

    void check_motors_overheat();

private:
    // DH parameters
    struct ArmConfig_t
    {
        float L_YAW_EXTEND;//最大长度
        float L_ARM_INITIAL;//初始长度
        float L_X2_RADIUS;//半径
    };
    ArmConfig_t armConfig = {0}; //公用
    float DH_matrix[6][4] = {0}; // home,d,a,alpha
};

extern Arm g_arm;

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif //__DRV_ARM_H
