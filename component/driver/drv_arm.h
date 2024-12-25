//
// Created by Henson on 2024-03-04.
//

#ifndef __DRV_ARM_H
#define __DRV_ARM_H

/*---------------------------- C++ Scope ---------------------------*/
#include "drv_actuator.h"
#include "drv_framework.h"
#include "drv_comm.h"
#include "robotics_math.h"
#include "drv_dof.h"
#include "user_lib.h"
#include "GlobalCfg.h"

const float RAD_TO_DEG = 57.295777754771045f;

#define X1_CNT_MAX  9
#define Y_CNT_MAX   5
#define X2_CNT_MAX  9

typedef struct{
    float dist;
    bool is_good;
    Eigen::Vector3f deg;
}MulSolu_search_t;

typedef struct{
    bool is_solution_changing;
    bool is_single_solution;
    bool is_concentric_double_solution;
    bool is_eccentric_double_solution;
}pose_select_bool_t;

class Arm {
public:
    Framework framework;
    Actuator actuator;
    Communication communication;
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
    bool is_fk_data_valid = false;
    bool is_all_peripheral_connect = false;
    bool is_none_of_peripheral_connect = true;
    bool is_motors_overheat = false;
    uint32_t arm_reset_num = 0;

    pose_select_bool_t pose_select_bool={.is_single_solution = true,.is_concentric_double_solution = false,.is_eccentric_double_solution = false};

    typedef struct{
        struct {
            Eigen::Matrix3f arm_zyx_rotMatrix;//radian  针对整个臂 考虑大yaw
            struct{
                Eigen::Vector3f xyz_mm;
                float euler_x1_deg;
                float euler_y_deg;
                float euler_x2_deg;
            }act;
            struct{
                Eigen::Vector3f xyz_mm;
                float extend_mm;
                float slide_mm;
                float uplift_mm;
                float arm_yaw_deg;
                float arm_pitch_deg;
            }framework;
            struct{
                Eigen::Matrix3f arm_yaw;
                Eigen::Matrix3f arm_pitch;
                Eigen::Matrix3f euler_x1;
                Eigen::Matrix3f euler_y;
                Eigen::Matrix3f euler_x2;

                Eigen::Matrix3f yaw_2_pitch;
                Eigen::Matrix3f yaw_2_pitch_2_x1;
                Eigen::Matrix3f yaw_2_pitch_2_x1_2_yx2;
            }rotation_matrix;
        }get;

        struct{
            Eigen::Matrix3f arm_zyx_rotMatrix;//radian  针对整个臂 考虑大yaw
            struct{
                Eigen::Vector3f euler_xyx_deg_pos;//同心
                Eigen::Vector3f euler_xyx_deg_neg;//同心
                Eigen::Matrix3f xyx_rotMatrix;//radian 不考虑大yaw 只考虑吸盘
                Eigen::Vector3f xyz_mm;//由设定值计算出的值
                float euler_x1_deg;//最终设定值
                float euler_y_deg;//最终设定值
                float euler_x2_deg;//最终设定值
            }act;
            struct{
                Eigen::Vector3f xyz_mm;
                float extend_mm;
                float slide_mm;
                float uplift_mm;
                float arm_yaw_deg;
                float arm_pitch_deg;
            }framework;
            struct{
                Eigen::Matrix3f arm_yaw;
                Eigen::Matrix3f arm_pitch;
                Eigen::Matrix3f euler_x1;
                Eigen::Matrix3f euler_y;
                Eigen::Matrix3f euler_x2;

                Eigen::Matrix3f yaw_2_pitch;
                Eigen::Matrix3f yaw_2_pitch_2_x1;
                Eigen::Matrix3f yaw_2_pitch_2_x1_2_yx2;
            }rotation_matrix;
        }set;
        //choose
        struct{
            MulSolu_search_t the_best_xyx;
            uint32_t solve_cnt;
        }mul_solution;
        //const
        Eigen::Vector3f const_yaw_2_pitch_mm;
        Eigen::Vector3f const_pitch_2_x1_mm;
        Eigen::Vector3f const_x1_2_yx2_mm;
        Eigen::Vector3f const_yx2_mm;
    }Kine_t;
    Kine_t kine={};

public:
    explicit Arm();
    void Arm_init();
    void kine_init();

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

    // set pose select
    // 复位后使用
    void set_pose_select_concentric_double_solution();
    void set_pose_select_eccentric_double_solution();
    void set_pose_select_single_solution();

    //dec
    void solveFk();//loop
    void solveIk();//loop
    void solve_multiple_solutions();
    void solveIkStandard();//loop
    void update_rotation_matrix();

    //comm
    void set_FK_data_to_communication();
    void send_FK_data_to_communication();//loop
    void set_communication_basic_data_to_rx_data();//loop
    void set_communication_more_data_to_rx_data();
    void set_communication_to_arm_pose();//loop
    void communication_custom_ctrl_data_to_encoder_direct_ctrl();

    //run
    void set_arm_move();
    void set_arm_pose(dof6_e dofE, float set_mm_deg);//接口
    void set_arm_arm_yaw_deg(float set_deg);
    void set_arm_arm_pitch_deg(float set_deg);
    bool is_arm_at_initial_pos();
    bool is_arm_reset_all_ok();

    void check_motors_overheat();

    void set_decomposition_for_custom_encoder_direct_ctrl();

private:
    // DH parameters
    struct ArmConfig_t {
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
