//
// Created by Henson on 2024-03-07.
//

#include "drv_arm.h"
#include "drv_Kinematic.h"

/* *
 * 大锥齿轮     40
 * 小锥齿轮     15
 * 同步带圆齿轮  20 20  或  24 20
 * 小圆齿轮     26
 * 大圆齿轮     44
 * */

// --------------------------- 底层解算 --------------------------- //

void Arm::kine_init() {
    kine.mul_solution.solve_cnt=0;
    kine.mul_solution.the_best_xyx.dist = 100000000.0f;
    set_pose_select_single_solution();
}

//同心
void Arm::set_pose_select_concentric_double_solution(){
    if(!pose_select_bool.is_concentric_double_solution){
        pose_select_bool.is_concentric_double_solution = true;
        pose_select_bool.is_eccentric_double_solution = false;
        pose_select_bool.is_single_solution = false;
        pose_select_bool.is_solution_changing = true;
    }
}

//单面
void Arm::set_pose_select_single_solution(){
    if(!pose_select_bool.is_single_solution){
        pose_select_bool.is_concentric_double_solution = false;
        pose_select_bool.is_eccentric_double_solution = false;
        pose_select_bool.is_single_solution = true;
        pose_select_bool.is_solution_changing = true;
    }
}

//偏心
void Arm::set_pose_select_eccentric_double_solution(){
    if(!pose_select_bool.is_eccentric_double_solution){
        pose_select_bool.is_concentric_double_solution = false;
        pose_select_bool.is_eccentric_double_solution = true;
        pose_select_bool.is_single_solution = false;
        pose_select_bool.is_solution_changing = true;
    }
}

void Arm::solve_multiple_solutions() {
    MulSolu_search_t xyx_pos,xyx_neg,xyx_final;
    Eigen::Vector3f using_xyx_deg;
    using_xyx_deg << kine.set.act.euler_x1_deg, kine.set.act.euler_y_deg, kine.set.act.euler_x2_deg;
    float dist_difference_weight = 0.92f, dist_origin_weight=0.05f, dist_x1_x2_cancellation_weight=0.03f, dist_x1_weight=0.03f;
    float x1_x2_dist_pos = 0.0f, x1_x2_dist_neg = 0.0f;
    //离远点距离->防止线断  两次变化间距->找最近解  x1 x2的和的绝对值->解决计算精度问题和相同解问题
    uint32_t i,j,k;
    for(i=0;i<X1_CNT_MAX;i++){
        for(j=0;j<Y_CNT_MAX;j++){
            for(k=0;k<X2_CNT_MAX;k++){
                //搜索
                xyx_pos.deg <<
                        kine.set.act.euler_xyx_deg_pos(0) + 360.0f * (float)((int32_t)i - (int32_t)(X1_CNT_MAX / 2)),
                        kine.set.act.euler_xyx_deg_pos(1) + 0.0f * (float)((int32_t)j - (int32_t)(Y_CNT_MAX / 2)),
                        kine.set.act.euler_xyx_deg_pos(2) + 360.0f * (float)((int32_t)k - (int32_t)(X2_CNT_MAX / 2));
                xyx_neg.deg <<
                        kine.set.act.euler_xyx_deg_neg(0) + 360.0f * (float)((int32_t)i - (int32_t)(X1_CNT_MAX / 2)),
                        kine.set.act.euler_xyx_deg_neg(1) + 0.0f * (float)((int32_t)j - (int32_t)(Y_CNT_MAX / 2)),
                        kine.set.act.euler_xyx_deg_neg(2) + 360.0f * (float)((int32_t)k - (int32_t)(X2_CNT_MAX / 2));

//                x1_x2_dist_pos = ABS(xyx_pos.deg(0)+xyx_pos.deg(2));
//                x1_x2_dist_neg = ABS(xyx_neg.deg(0)+xyx_neg.deg(2));
                arm_sqrt_f32(ABS(xyx_pos.deg(0)+xyx_pos.deg(2)),&x1_x2_dist_pos);
                arm_sqrt_f32(ABS(xyx_neg.deg(0)+xyx_neg.deg(2)),&x1_x2_dist_neg);

                if(pose_select_bool.is_solution_changing){//这里不是最近距离, 应该是有条件的最近距离
                    //pose select bool
                    //双解处理限幅
                    if(pose_select_bool.is_concentric_double_solution) { //修改为不改变
                        //双解 限幅
                        if(xyx_pos.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_pos.deg(0)<=ACTUATOR_X1_MAX_DEG
                           && xyx_pos.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_pos.deg(1)<=ACTUATOR_Y_MAX_DEG
                           && xyx_pos.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_pos.deg(2)<=ACTUATOR_X2_MAX_DEG)
                        {
                            xyx_pos.dist = dist_difference_weight*(xyx_pos.deg - using_xyx_deg).norm()
                                           + dist_origin_weight*xyx_pos.deg.norm()
                                           + dist_x1_x2_cancellation_weight*x1_x2_dist_pos;
                            xyx_pos.is_good = true;
                        }else{
                            xyx_pos.is_good = false;
                        }

                        //限幅
                        if(xyx_neg.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_neg.deg(0)<=ACTUATOR_X1_MAX_DEG
                           && xyx_neg.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_neg.deg(1)<=ACTUATOR_Y_MAX_DEG
                           && xyx_neg.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_neg.deg(2)<=ACTUATOR_X2_MAX_DEG)
                        {
                            xyx_neg.dist = dist_difference_weight*(xyx_neg.deg - using_xyx_deg).norm()
                                           + dist_origin_weight*xyx_neg.deg.norm()
                                           + dist_x1_x2_cancellation_weight*x1_x2_dist_neg;
                            xyx_neg.is_good = true;
                        }else{
                            xyx_neg.is_good = false;
                        }
                    }
                    else if(pose_select_bool.is_eccentric_double_solution) {//偏心
                        float temp = xyx_neg.deg(0);
                        while(temp>180.0f){
                            temp = temp-360.0f;
                        }
                        while(temp<-180.0f){
                            temp = temp+360.0f;
                        }

                        if(!(temp<=90.0f && temp>=-90.0f)){
                            if(xyx_neg.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_neg.deg(0)<=ACTUATOR_X1_MAX_DEG
                               && xyx_neg.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_neg.deg(1)<=ACTUATOR_Y_MAX_DEG
                               && xyx_neg.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_neg.deg(2)<=ACTUATOR_X2_MAX_DEG)
                            {
                                xyx_neg.dist = dist_difference_weight*(xyx_neg.deg - using_xyx_deg).norm()
                                               + dist_origin_weight*xyx_neg.deg.norm()
                                               + dist_x1_x2_cancellation_weight*x1_x2_dist_neg;
                                xyx_neg.is_good = true;
                            }else{
                                xyx_neg.is_good = false;
                            }
                        }else{
                            xyx_neg.is_good = false;
                        }

                        temp = xyx_pos.deg(0);
                        while(temp>180.0f){
                            temp = temp-360.0f;
                        }
                        while(temp<-180.0f){
                            temp = temp+360.0f;
                        }

                        if(!(temp<=90.0f && temp>=-90.0f)){
                            if(xyx_pos.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_pos.deg(0)<=ACTUATOR_X1_MAX_DEG
                               && xyx_pos.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_pos.deg(1)<=ACTUATOR_Y_MAX_DEG
                               && xyx_pos.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_pos.deg(2)<=ACTUATOR_X2_MAX_DEG)
                            {
                                xyx_pos.dist = dist_difference_weight*(xyx_pos.deg - using_xyx_deg).norm()
                                               + dist_origin_weight *xyx_pos.deg.norm()
                                               + dist_x1_x2_cancellation_weight*x1_x2_dist_pos;
                                xyx_pos.is_good = true;
                            }else{
                                xyx_pos.is_good = false;
                            }
                        }else{
                            xyx_pos.is_good = false;
                        }
                    }
                    else {//单面
                        //单解处理限幅
                        //限幅 neg 保证单面不碰撞
                        if(xyx_neg.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_neg.deg(0)<=ACTUATOR_X1_MAX_DEG
                           && xyx_neg.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_neg.deg(1)<=ACTUATOR_Y_MAX_DEG
                           && xyx_neg.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_neg.deg(2)<=ACTUATOR_X2_MAX_DEG)
                        {
                            xyx_neg.dist = dist_difference_weight*(xyx_neg.deg - using_xyx_deg).norm()
                                           + dist_origin_weight*xyx_neg.deg.norm()
                                           + dist_x1_x2_cancellation_weight*x1_x2_dist_neg;
                            xyx_neg.is_good = true;
                        }else{
                            xyx_neg.is_good = false;
                        }
                        xyx_pos.is_good = false;
                    }
                }
                else//未发生模式切换
                {
                    //pose select bool
                    //双解处理限幅
                    if(pose_select_bool.is_concentric_double_solution)
                    {//同心的双解处理
                        //双解 限幅
                        if(xyx_pos.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_pos.deg(0)<=ACTUATOR_X1_MAX_DEG
                           && xyx_pos.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_pos.deg(1)<=ACTUATOR_Y_MAX_DEG
                           && xyx_pos.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_pos.deg(2)<=ACTUATOR_X2_MAX_DEG)
                        {
                            xyx_pos.dist = dist_difference_weight*(xyx_pos.deg - using_xyx_deg).norm()
                                           + dist_origin_weight*xyx_pos.deg.norm()
                                           + dist_x1_x2_cancellation_weight*x1_x2_dist_pos;
                            xyx_pos.is_good = true;
                        }else{
                            xyx_pos.is_good = false;
                        }

                        //限幅
                        if(xyx_neg.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_neg.deg(0)<=ACTUATOR_X1_MAX_DEG
                           && xyx_neg.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_neg.deg(1)<=ACTUATOR_Y_MAX_DEG
                           && xyx_neg.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_neg.deg(2)<=ACTUATOR_X2_MAX_DEG)
                        {
                            xyx_neg.dist = dist_difference_weight*(xyx_neg.deg - using_xyx_deg).norm()
                                           + dist_origin_weight*xyx_neg.deg.norm()
                                           + dist_x1_x2_cancellation_weight*x1_x2_dist_neg;
                            xyx_neg.is_good = true;
                        }else{
                            xyx_neg.is_good = false;
                        }
                    }
                    else if(pose_select_bool.is_eccentric_double_solution){//偏心
                            //正常解算
                            if(xyx_neg.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_neg.deg(0)<=ACTUATOR_X1_MAX_DEG
                               && xyx_neg.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_neg.deg(1)<=ACTUATOR_Y_MAX_DEG
                               && xyx_neg.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_neg.deg(2)<=ACTUATOR_X2_MAX_DEG)
                            {
                                xyx_neg.dist = dist_difference_weight*(xyx_neg.deg - using_xyx_deg).norm()
                                               + dist_origin_weight*xyx_neg.deg.norm()
                                               + dist_x1_x2_cancellation_weight*x1_x2_dist_neg;
                                xyx_neg.is_good = true;
                            }else{
                                xyx_neg.is_good = false;
                            }

                            //正常解算
                            if(xyx_pos.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_pos.deg(0)<=ACTUATOR_X1_MAX_DEG
                               && xyx_pos.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_pos.deg(1)<=ACTUATOR_Y_MAX_DEG
                               && xyx_pos.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_pos.deg(2)<=ACTUATOR_X2_MAX_DEG)
                            {
                                xyx_pos.dist = dist_difference_weight*(xyx_pos.deg - using_xyx_deg).norm()
                                               + dist_origin_weight*xyx_pos.deg.norm()
                                               + dist_x1_x2_cancellation_weight*x1_x2_dist_pos;
                                xyx_pos.is_good = true;
                            }else{
                                xyx_pos.is_good = false;
                            }
                    }
                    else
                    {   //单面
                        //单解处理限幅
                        //限幅 neg 保证单面不碰撞
                        if(xyx_neg.deg(0)>=ACTUATOR_X1_MIN_DEG && xyx_neg.deg(0)<=ACTUATOR_X1_MAX_DEG
                           && xyx_neg.deg(1)>=ACTUATOR_Y_MIN_DEG && xyx_neg.deg(1)<=ACTUATOR_Y_MAX_DEG
                           && xyx_neg.deg(2)>=ACTUATOR_X2_MIN_DEG && xyx_neg.deg(2)<=ACTUATOR_X2_MAX_DEG)
                        {
                            xyx_neg.dist = dist_difference_weight*(xyx_neg.deg - using_xyx_deg).norm()
                                           + dist_origin_weight*xyx_neg.deg.norm()
                                           + dist_x1_x2_cancellation_weight*x1_x2_dist_neg;
                            xyx_neg.is_good = true;
                        }else{
                            xyx_neg.is_good = false;
                        }
                        xyx_pos.is_good = false;
                    }
                }

                //比较
                if(xyx_pos.is_good && xyx_neg.is_good){
                    if(xyx_pos.dist<xyx_neg.dist ){
                        xyx_final = xyx_pos;
                    }else{
                        xyx_final = xyx_neg;
                    }
                }else if(xyx_pos.is_good && !xyx_neg.is_good){
                    xyx_final = xyx_pos;
                }else if(!xyx_pos.is_good && xyx_neg.is_good){
                    xyx_final = xyx_neg;
                }else{//全都非good
                    xyx_final.is_good = false;
                }

                //判断
                if(xyx_final.is_good) {
                    if(xyx_final.dist < kine.mul_solution.the_best_xyx.dist){
                        kine.mul_solution.the_best_xyx.deg = xyx_final.deg;
                        kine.mul_solution.the_best_xyx.dist = xyx_final.dist;
                        kine.mul_solution.the_best_xyx.is_good = true;//代表进入此循环了
                    }
                }

            }//k
        }//j
    }//i

    if(kine.mul_solution.the_best_xyx.is_good) {
        kine.set.act.euler_x1_deg = kine.mul_solution.the_best_xyx.deg(0);
        kine.set.act.euler_y_deg = kine.mul_solution.the_best_xyx.deg(1);
        kine.set.act.euler_x2_deg = kine.mul_solution.the_best_xyx.deg(2);
        kine.mul_solution.solve_cnt++;
    }
    kine.mul_solution.the_best_xyx.dist = 100000000.0f;
    kine.mul_solution.the_best_xyx.is_good = false;

    pose_select_bool.is_solution_changing = false;
}


void Arm::update_rotation_matrix() {
    __NOP();
}

void Arm::solveIk() {
    //set
    kine.set.framework.arm_yaw_deg = set_arm_yaw_deg;
    kine.set.framework.arm_pitch_deg = set_arm_pitch_deg;

    kine.set.rotation_matrix.arm_yaw = Eigen::AngleAxisf(kine.set.framework.arm_yaw_deg / 180.0f * PI, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    kine.set.rotation_matrix.arm_pitch = Eigen::AngleAxisf(kine.set.framework.arm_pitch_deg / 180.0f * PI, Eigen::Vector3f::UnitY()).toRotationMatrix();
    kine.set.rotation_matrix.euler_x1 = Eigen::AngleAxisf(kine.set.act.euler_x1_deg / 180.0f * PI, Eigen::Vector3f::UnitX()).toRotationMatrix();
    kine.set.rotation_matrix.euler_y = Eigen::AngleAxisf(kine.set.act.euler_y_deg / 180.0f * PI, Eigen::Vector3f::UnitY()).toRotationMatrix();
    kine.set.rotation_matrix.euler_x2 = Eigen::AngleAxisf(kine.set.act.euler_x2_deg / 180.0f * PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

    kine.set.rotation_matrix.yaw_2_pitch = kine.set.rotation_matrix.arm_yaw * kine.set.rotation_matrix.arm_pitch;
    kine.set.rotation_matrix.yaw_2_pitch_2_x1 = kine.set.rotation_matrix.yaw_2_pitch * kine.set.rotation_matrix.euler_x1;
    kine.set.rotation_matrix.yaw_2_pitch_2_x1_2_yx2 = kine.set.rotation_matrix.yaw_2_pitch_2_x1 * kine.set.rotation_matrix.euler_y * kine.set.rotation_matrix.euler_x2;
    //set

    //altitude
    // zyx -> zyx_rotation -> zyx_rotation = z * y * xyx_rotation
    kine.set.arm_zyx_rotMatrix = Eigen::AngleAxisf(set.yaw_deg / 180.0f * PI, Eigen::Vector3f::UnitZ())
                                 * Eigen::AngleAxisf(set.pitch_deg/180.0f*PI, Eigen::Vector3f::UnitY())
                                 * Eigen::AngleAxisf(set.roll_deg/180.0f*PI, Eigen::Vector3f::UnitX());
    //dec
    kine.set.act.xyx_rotMatrix = (kine.set.rotation_matrix.arm_pitch.transpose() * kine.set.rotation_matrix.arm_yaw.transpose() * kine.set.arm_zyx_rotMatrix);
    kine.set.act.xyx_rotMatrix.transposeInPlace();

    //y 0-180 同心下
    kine.set.act.euler_xyx_deg_pos = (rotMatrix_to_euler_xyx_pos(kine.set.act.xyx_rotMatrix.data()).array()) / PI * 180.0f;
    //y -180-0 同心下
    kine.set.act.euler_xyx_deg_neg = (rotMatrix_to_euler_xyx_neg(kine.set.act.xyx_rotMatrix.data()).array()) / PI * 180.0f;

    //position
    Eigen::Vector3f set_arm_xyz_vec,fb_set_act_xyz_mm;
    kine.set.act.xyz_mm = kine.set.rotation_matrix.arm_yaw * kine.const_yaw_2_pitch_mm
                                     + kine.set.rotation_matrix.yaw_2_pitch * kine.const_pitch_2_x1_mm
                                     + kine.set.rotation_matrix.yaw_2_pitch_2_x1 * kine.const_x1_2_yx2_mm
                                     + kine.set.rotation_matrix.yaw_2_pitch_2_x1_2_yx2 * kine.const_yx2_mm;

    fb_set_act_xyz_mm = kine.set.act.xyz_mm;
    set_arm_xyz_vec << set.x_mm,set.y_mm,set.z_mm;

    //这里用的是get的x1就是慢速移动机械臂, 若是set就是直接过去, 没有偏心了先直接过去
    kine.set.framework.xyz_mm = set_arm_xyz_vec - kine.set.act.xyz_mm;

    kine.set.framework.extend_mm = kine.set.framework.xyz_mm(0);
    kine.set.framework.slide_mm = kine.set.framework.xyz_mm(1);
    kine.set.framework.uplift_mm = kine.set.framework.xyz_mm(2);

    arm_sqrt_f32(
            fb_set_act_xyz_mm(0) * fb_set_act_xyz_mm(0) + fb_set_act_xyz_mm(1) * fb_set_act_xyz_mm(1),
            &communication.can.tx_fb_data.total_arm_xy_dist_mm);
    VAL_LIMIT(communication.can.tx_fb_data.total_arm_xy_dist_mm,10,800);//todo限幅
    float temp_yaw_rad = 0.0f;
    arm_atan2_f32(fb_set_act_xyz_mm(1), fb_set_act_xyz_mm(0), &temp_yaw_rad);
    communication.can.tx_fb_data.total_arm_xy_yaw_deg = temp_yaw_rad/PI*180.0f;
    VAL_LIMIT(communication.can.tx_fb_data.total_arm_xy_yaw_deg,-90,90);
}

void Arm::solveFk()
{
    kine.get.act.euler_x1_deg = actuator.get_actuator_x1_deg();
    kine.get.act.euler_y_deg = actuator.get_actuator_y_deg();
    kine.get.act.euler_x2_deg = actuator.get_actuator_x2_deg();

    kine.get.framework.extend_mm = framework.get_framework_extend_mm();
    kine.get.framework.slide_mm = framework.get_framework_slide_mm();
    kine.get.framework.uplift_mm = framework.get_framework_uplift_mm();
    kine.get.framework.arm_yaw_deg = framework.get_framework_arm_yaw_deg();
    kine.get.framework.arm_pitch_deg = framework.get_framework_arm_pitch_deg();

    //get
    kine.get.rotation_matrix.arm_yaw = Eigen::AngleAxisf(kine.get.framework.arm_yaw_deg / 180.0f * PI, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    kine.get.rotation_matrix.arm_pitch = Eigen::AngleAxisf(kine.get.framework.arm_pitch_deg / 180.0f * PI, Eigen::Vector3f::UnitY()).toRotationMatrix();
    kine.get.rotation_matrix.euler_x1 = Eigen::AngleAxisf(kine.get.act.euler_x1_deg / 180.0f * PI, Eigen::Vector3f::UnitX()).toRotationMatrix();
    kine.get.rotation_matrix.euler_y = Eigen::AngleAxisf(kine.get.act.euler_y_deg / 180.0f * PI, Eigen::Vector3f::UnitY()).toRotationMatrix();
    kine.get.rotation_matrix.euler_x2 = Eigen::AngleAxisf(kine.get.act.euler_x2_deg / 180.0f * PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

    kine.get.rotation_matrix.yaw_2_pitch = kine.get.rotation_matrix.arm_yaw * kine.get.rotation_matrix.arm_pitch;
    kine.get.rotation_matrix.yaw_2_pitch_2_x1 = kine.get.rotation_matrix.yaw_2_pitch * kine.get.rotation_matrix.euler_x1;
    kine.get.rotation_matrix.yaw_2_pitch_2_x1_2_yx2 = kine.get.rotation_matrix.yaw_2_pitch_2_x1 * kine.get.rotation_matrix.euler_y * kine.get.rotation_matrix.euler_x2;
    //get

    //get xyz
    Eigen::Vector3f get_arm_xyz_vec;
    kine.get.framework.xyz_mm << kine.get.framework.extend_mm,kine.get.framework.slide_mm,kine.get.framework.uplift_mm;
    kine.get.act.xyz_mm = kine.get.rotation_matrix.arm_yaw * kine.const_yaw_2_pitch_mm
                          + kine.get.rotation_matrix.yaw_2_pitch * kine.const_pitch_2_x1_mm
                          + kine.get.rotation_matrix.yaw_2_pitch_2_x1 * kine.const_x1_2_yx2_mm
                          + kine.get.rotation_matrix.yaw_2_pitch_2_x1_2_yx2 * kine.const_yx2_mm;

    get_arm_xyz_vec = kine.get.framework.xyz_mm + kine.get.act.xyz_mm;

    get.x_mm = get_arm_xyz_vec(0);
    get.y_mm = get_arm_xyz_vec(1);
    get.z_mm = get_arm_xyz_vec(2);

    VAL_LIMIT(get.x_mm,0-ARM_X_MM_OFFSET,1024-ARM_X_MM_OFFSET);
    VAL_LIMIT(get.y_mm,0-ARM_Y_MM_OFFSET,1024-ARM_Y_MM_OFFSET);
    VAL_LIMIT(get.z_mm,0-ARM_Z_MM_OFFSET,1024-ARM_Z_MM_OFFSET);

    //get pose
    Eigen::Vector3f get_arm_zyx_pose_deg;

    kine.get.arm_zyx_rotMatrix = kine.get.rotation_matrix.yaw_2_pitch_2_x1_2_yx2;
    kine.get.arm_zyx_rotMatrix.transposeInPlace();//防止data顺序为转置
    get_arm_zyx_pose_deg = rotMatrix_to_euler_zyx(kine.get.arm_zyx_rotMatrix.data()).array() / PI * 180.0f;
    // todo 因为这个函数有问题,没有考虑到多解问题的处理，所以 yaw pitch roll不准, 后续交给你来做啦~

    get.yaw_deg = get_arm_zyx_pose_deg(0);
    get.pitch_deg = get_arm_zyx_pose_deg(1);
    get.roll_deg = get_arm_zyx_pose_deg(2);

    if(is_arm_offset_ok()){
        is_fk_data_valid = true;
    }
}


void Arm::communication_custom_ctrl_data_to_encoder_direct_ctrl(){
    taskENTER_CRITICAL();
    communication.can.rx_custom_ctrl_data.rcv_cnt++;

    communication.can.rx_custom_ctrl_data.frame_x_mm = communication.can.rx_custom_raw.frame_x_mm;
    communication.can.rx_custom_ctrl_data.frame_y_mm = communication.can.rx_custom_raw.frame_y_mm;
    communication.can.rx_custom_ctrl_data.frame_z_mm = communication.can.rx_custom_raw.frame_z_mm;

    if(communication.can.rx_custom_ctrl_data.rcv_cnt<15){
        //euler_x1
        communication.can.rx_custom_ctrl_data.euler_x1.spill_cnt = 0;
        communication.can.rx_custom_ctrl_data.euler_x1.round = communication.can.rx_custom_raw.euler_x1_deg;
        communication.can.rx_custom_ctrl_data.euler_x1.round /= 360.0f;
        VAL_LIMIT(communication.can.rx_custom_ctrl_data.euler_x1.round,-0.5f,0.5f);
        communication.can.rx_custom_ctrl_data.euler_x1.offset_round = communication.can.rx_custom_ctrl_data.euler_x1.round;

        //euler_y
        communication.can.rx_custom_ctrl_data.euler_y.spill_cnt = 0;
        communication.can.rx_custom_ctrl_data.euler_y.round = communication.can.rx_custom_raw.euler_y_deg;
        communication.can.rx_custom_ctrl_data.euler_y.round /= 360.0f;
        VAL_LIMIT(communication.can.rx_custom_ctrl_data.euler_y.round,-0.5f,0.5f);
        communication.can.rx_custom_ctrl_data.euler_y.offset_round = communication.can.rx_custom_ctrl_data.euler_y.round;

        //euler_x2
        communication.can.rx_custom_ctrl_data.euler_x2.spill_cnt = 0;
        communication.can.rx_custom_ctrl_data.euler_x2.round = communication.can.rx_custom_raw.euler_x2_deg;
        communication.can.rx_custom_ctrl_data.euler_x2.round /= 360.0f;
        VAL_LIMIT(communication.can.rx_custom_ctrl_data.euler_x2.round,-0.5f,0.5f);
        communication.can.rx_custom_ctrl_data.euler_x2.offset_round = communication.can.rx_custom_ctrl_data.euler_x2.round;

        //arm_yaw 这个要除以2
        communication.can.rx_custom_ctrl_data.arm_yaw.spill_cnt = 0;
        communication.can.rx_custom_ctrl_data.arm_yaw.round = communication.can.rx_custom_raw.arm_yaw_deg;
        communication.can.rx_custom_ctrl_data.arm_yaw.round /= 2.0f;
        communication.can.rx_custom_ctrl_data.arm_yaw.round /= 360.0f;
        VAL_LIMIT(communication.can.rx_custom_ctrl_data.arm_yaw.round,-0.5f,0.5f);
        communication.can.rx_custom_ctrl_data.arm_yaw.offset_round = communication.can.rx_custom_ctrl_data.arm_yaw.round;
        //总控
        communication.can.rx_custom_ctrl_data.is_offset_ok = false;
    }else{
        //euler_x1
        communication.can.rx_custom_ctrl_data.euler_x1.last_round = communication.can.rx_custom_ctrl_data.euler_x1.round;
        communication.can.rx_custom_ctrl_data.euler_x1.round = communication.can.rx_custom_raw.euler_x1_deg;
        communication.can.rx_custom_ctrl_data.euler_x1.round /= 360.0f;
        VAL_LIMIT(communication.can.rx_custom_ctrl_data.euler_x1.round,-0.5f,0.5f);//一圈是0.5-(-0.5)
        if (communication.can.rx_custom_ctrl_data.euler_x1.round - communication.can.rx_custom_ctrl_data.euler_x1.last_round > 0.5f){//一圈是(0.5-(-0.5))/2
            communication.can.rx_custom_ctrl_data.euler_x1.spill_cnt--;
        }
        else if (communication.can.rx_custom_ctrl_data.euler_x1.round - communication.can.rx_custom_ctrl_data.euler_x1.last_round < -0.5f){
            communication.can.rx_custom_ctrl_data.euler_x1.spill_cnt++;
        }
        communication.can.rx_custom_ctrl_data.euler_x1.total_rounds = (float)communication.can.rx_custom_ctrl_data.euler_x1.spill_cnt
                + communication.can.rx_custom_ctrl_data.euler_x1.round
                - communication.can.rx_custom_ctrl_data.euler_x1.offset_round;
        communication.can.rx_custom_ctrl_data.euler_x1.total_rounds_without_offset = (float)communication.can.rx_custom_ctrl_data.euler_x1.spill_cnt
                + communication.can.rx_custom_ctrl_data.euler_x1.round;

        //euler_y
        communication.can.rx_custom_ctrl_data.euler_y.last_round = communication.can.rx_custom_ctrl_data.euler_y.round;
        communication.can.rx_custom_ctrl_data.euler_y.round = communication.can.rx_custom_raw.euler_y_deg;
        communication.can.rx_custom_ctrl_data.euler_y.round /= 360.0f;
        VAL_LIMIT(communication.can.rx_custom_ctrl_data.euler_y.round,-0.5f,0.5f);
        if (communication.can.rx_custom_ctrl_data.euler_y.round - communication.can.rx_custom_ctrl_data.euler_y.last_round > 0.5f){
            communication.can.rx_custom_ctrl_data.euler_y.spill_cnt--;
        }
        else if (communication.can.rx_custom_ctrl_data.euler_y.round - communication.can.rx_custom_ctrl_data.euler_y.last_round < -0.5f){
            communication.can.rx_custom_ctrl_data.euler_y.spill_cnt++;
        }
        communication.can.rx_custom_ctrl_data.euler_y.total_rounds = (float)communication.can.rx_custom_ctrl_data.euler_y.spill_cnt
                + communication.can.rx_custom_ctrl_data.euler_y.round
                - communication.can.rx_custom_ctrl_data.euler_y.offset_round;
        communication.can.rx_custom_ctrl_data.euler_y.total_rounds_without_offset = (float)communication.can.rx_custom_ctrl_data.euler_y.spill_cnt
                + communication.can.rx_custom_ctrl_data.euler_y.round;

        //euler_x2
        communication.can.rx_custom_ctrl_data.euler_x2.last_round = communication.can.rx_custom_ctrl_data.euler_x2.round;
        communication.can.rx_custom_ctrl_data.euler_x2.round = communication.can.rx_custom_raw.euler_x2_deg;
        communication.can.rx_custom_ctrl_data.euler_x2.round /= 360.0f;
        VAL_LIMIT(communication.can.rx_custom_ctrl_data.euler_x2.round,-0.5f,0.5f);
        if (communication.can.rx_custom_ctrl_data.euler_x2.round - communication.can.rx_custom_ctrl_data.euler_x2.last_round > 0.5f){
            communication.can.rx_custom_ctrl_data.euler_x2.spill_cnt--;
        }
        else if (communication.can.rx_custom_ctrl_data.euler_x2.round - communication.can.rx_custom_ctrl_data.euler_x2.last_round < -0.5f){
            communication.can.rx_custom_ctrl_data.euler_x2.spill_cnt++;
        }
        communication.can.rx_custom_ctrl_data.euler_x2.total_rounds = (float)communication.can.rx_custom_ctrl_data.euler_x2.spill_cnt
                                                                      + communication.can.rx_custom_ctrl_data.euler_x2.round
                                                                      - communication.can.rx_custom_ctrl_data.euler_x2.offset_round;
        communication.can.rx_custom_ctrl_data.euler_x2.total_rounds_without_offset = (float)communication.can.rx_custom_ctrl_data.euler_x2.spill_cnt
                + communication.can.rx_custom_ctrl_data.euler_x2.round;

        //arm_yaw
        communication.can.rx_custom_ctrl_data.arm_yaw.last_round = communication.can.rx_custom_ctrl_data.arm_yaw.round;
        communication.can.rx_custom_ctrl_data.arm_yaw.round = communication.can.rx_custom_raw.arm_yaw_deg;
        communication.can.rx_custom_ctrl_data.arm_yaw.round /= 360.0f;
        VAL_LIMIT(communication.can.rx_custom_ctrl_data.arm_yaw.round,-0.5f,0.5f);
        if (communication.can.rx_custom_ctrl_data.arm_yaw.round - communication.can.rx_custom_ctrl_data.arm_yaw.last_round > 0.5f){
            communication.can.rx_custom_ctrl_data.arm_yaw.spill_cnt--;
        }
        else if (communication.can.rx_custom_ctrl_data.arm_yaw.round - communication.can.rx_custom_ctrl_data.arm_yaw.last_round < -0.5f){
            communication.can.rx_custom_ctrl_data.arm_yaw.spill_cnt++;
        }
        communication.can.rx_custom_ctrl_data.arm_yaw.total_rounds = (float)communication.can.rx_custom_ctrl_data.arm_yaw.spill_cnt
                                                                      + communication.can.rx_custom_ctrl_data.arm_yaw.round
                                                                      - communication.can.rx_custom_ctrl_data.arm_yaw.offset_round;
        communication.can.rx_custom_ctrl_data.arm_yaw.total_rounds_without_offset = (float)communication.can.rx_custom_ctrl_data.arm_yaw.spill_cnt
                + communication.can.rx_custom_ctrl_data.arm_yaw.round;

        //总控
        communication.can.rx_custom_ctrl_data.is_offset_ok = true;
    }
    taskEXIT_CRITICAL();
}

//直接设定数值
void Arm::set_decomposition_for_custom_encoder_direct_ctrl() {
    if(communication.can.rx_custom_ctrl_data.is_using_custom_ctrl && communication.can.rx_custom_ctrl_data.is_offset_ok) {
        kine.set.framework.extend_mm = communication.can.rx_custom_ctrl_data.frame_x_mm;
        kine.set.framework.slide_mm = communication.can.rx_custom_ctrl_data.frame_y_mm;
        kine.set.framework.uplift_mm = communication.can.rx_custom_ctrl_data.frame_z_mm;

        kine.set.act.euler_x1_deg = communication.can.rx_custom_ctrl_data.euler_x1.total_rounds_without_offset*360.0f;
        kine.set.act.euler_y_deg = communication.can.rx_custom_ctrl_data.euler_y.total_rounds_without_offset*360.0f;
        kine.set.act.euler_x2_deg = communication.can.rx_custom_ctrl_data.euler_x2.total_rounds_without_offset*360.0f;
        kine.set.framework.arm_yaw_deg = communication.can.rx_custom_ctrl_data.arm_yaw.total_rounds_without_offset*360.0f;

        kine.set.framework.arm_pitch_deg = set_arm_pitch_deg;//还是控制pitch
    }
}


// --------------------------------------------- 其他人 --------------------------------------------- //

DOF6Kinematic::DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT)
        : armConfig(ArmConfig_t{L_BS, D_BS, L_AM, L_FA, D_EW, L_WT})
{
    float tmp_DH_matrix[6][4] = {
            {0.0f,            armConfig.L_BASE,    armConfig.D_BASE, -(float) M_PI_2},
            {-(float) M_PI_2, 0.0f,                armConfig.L_ARM,  0.0f},
            {(float) M_PI_2,  armConfig.D_ELBOW,   0.0f,             (float) M_PI_2},
            {0.0f,            armConfig.L_FOREARM, 0.0f,             -(float) M_PI_2},
            {0.0f,            0.0f,                0.0f,             (float) M_PI_2},
            {0.0f,            armConfig.L_WRIST, 0.0f, 0.0f}
    };
    memcpy(DH_matrix, tmp_DH_matrix, sizeof(tmp_DH_matrix));

    float tmp_L1_bs[3] = {armConfig.D_BASE, -armConfig.L_BASE, 0.0f};
    memcpy(L1_base, tmp_L1_bs, sizeof(tmp_L1_bs));
    float tmp_L2_se[3] = {armConfig.L_ARM, 0.0f, 0.0f};
    memcpy(L2_arm, tmp_L2_se, sizeof(tmp_L2_se));
    float tmp_L3_ew[3] = {-armConfig.D_ELBOW, 0.0f, armConfig.L_FOREARM};
    memcpy(L3_elbow, tmp_L3_ew, sizeof(tmp_L3_ew));
    float tmp_L6_wt[3] = {0.0f, 0.0f, armConfig.L_WRIST};
    memcpy(L6_wrist, tmp_L6_wt, sizeof(tmp_L6_wt));

    l_se_2 = armConfig.L_ARM * armConfig.L_ARM;
    l_se = armConfig.L_ARM;
    l_ew_2 = armConfig.L_FOREARM * armConfig.L_FOREARM + armConfig.D_ELBOW * armConfig.D_ELBOW;
    l_ew = 0;
    atan_e = 0;
}

bool
DOF6Kinematic::SolveFK(const DOF6Kinematic::Joint6D_t &_inputJoint6D, DOF6Kinematic::Pose6D_t &_outputPose6D)
{
    float q_in[6];
    float q[6];
    float cosq, sinq;
    float cosa, sina;
    float P06[6];
    float R06[9];
    float R[6][9];
    float R02[9];
    float R03[9];
    float R04[9];
    float R05[9];
    float L0_bs[3];
    float L0_se[3];
    float L0_ew[3];
    float L0_wt[3];

    for (int i = 0; i < 6; i++)
        q_in[i] = _inputJoint6D.a[i] / RAD_TO_DEG;

    for (int i = 0; i < 6; i++)
    {
        q[i] = q_in[i] + DH_matrix[i][0];
        cosq = cosf(q[i]);
        sinq = sinf(q[i]);
        cosa = cosf(DH_matrix[i][3]);
        sina = sinf(DH_matrix[i][3]);

        R[i][0] = cosq;
        R[i][1] = -cosa * sinq;
        R[i][2] = sina * sinq;
        R[i][3] = sinq;
        R[i][4] = cosa * cosq;
        R[i][5] = -sina * cosq;
        R[i][6] = 0.0f;
        R[i][7] = sina;
        R[i][8] = cosa;
    }

    MatMultiply(R[0], R[1], R02, 3, 3, 3);
    MatMultiply(R02, R[2], R03, 3, 3, 3);
    MatMultiply(R03, R[3], R04, 3, 3, 3);
    MatMultiply(R04, R[4], R05, 3, 3, 3);
    MatMultiply(R05, R[5], R06, 3, 3, 3);

    MatMultiply(R[0], L1_base, L0_bs, 3, 3, 1);
    MatMultiply(R02, L2_arm, L0_se, 3, 3, 1);
    MatMultiply(R03, L3_elbow, L0_ew, 3, 3, 1);
    MatMultiply(R06, L6_wrist, L0_wt, 3, 3, 1);

    for (int i = 0; i < 3; i++)
        P06[i] = L0_bs[i] + L0_se[i] + L0_ew[i] + L0_wt[i];

    RotMatToEulerAngle(R06, &(P06[3]));

    _outputPose6D.X = P06[0];
    _outputPose6D.Y = P06[1];
    _outputPose6D.Z = P06[2];
    _outputPose6D.A = P06[3] * RAD_TO_DEG;
    _outputPose6D.B = P06[4] * RAD_TO_DEG;
    _outputPose6D.C = P06[5] * RAD_TO_DEG;
    memcpy(_outputPose6D.R, R06, 9 * sizeof(float));

    return true;
}

bool DOF6Kinematic::SolveIK(const DOF6Kinematic::Pose6D_t &_inputPose6D, const Joint6D_t &_lastJoint6D,
                            DOF6Kinematic::IKSolves_t &_outputSolves)
{
    float qs[2];
    float qa[2][2];
    float qw[2][3];
    float cosqs, sinqs;
    float cosqa[2], sinqa[2];
    float cosqw, sinqw;
    float P06[6];
    float R06[9];
    float P0_w[3];
    float P1_w[3];
    float L0_wt[3];
    float L1_sw[3];
    float R10[9];
    float R31[9];
    float R30[9];
    float R36[9];
    float l_sw_2, l_sw, atan_a, acos_a, acos_e;

    int ind_arm, ind_elbow, ind_wrist;
    int i;

    if (0 == l_ew)
    {
        l_ew = sqrtf(l_ew_2);
        atan_e = atanf(armConfig.D_ELBOW / armConfig.L_FOREARM);
    }

    P06[0] = _inputPose6D.X / 1000.0f;
    P06[1] = _inputPose6D.Y / 1000.0f;
    P06[2] = _inputPose6D.Z / 1000.0f;
    if (!_inputPose6D.hasR)
    {
        P06[3] = _inputPose6D.A / RAD_TO_DEG;
        P06[4] = _inputPose6D.B / RAD_TO_DEG;
        P06[5] = _inputPose6D.C / RAD_TO_DEG;
        zyxEulerAngleToRotMat(&(P06[3]), R06);
    } else
    {
        memcpy(R06, _inputPose6D.R, 9 * sizeof(float));
    }
    for (i = 0; i < 2; i++)
    {
        qs[i] = _lastJoint6D.a[0];
        qa[i][0] = _lastJoint6D.a[1];
        qa[i][1] = _lastJoint6D.a[2];
        qw[i][0] = _lastJoint6D.a[3];
        qw[i][1] = _lastJoint6D.a[4];
        qw[i][2] = _lastJoint6D.a[5];
    }
    MatMultiply(R06, L6_wrist, L0_wt, 3, 3, 1);
    for (i = 0; i < 3; i++)
    {
        P0_w[i] = P06[i] - L0_wt[i];
    }
    if (sqrt(P0_w[0] * P0_w[0] + P0_w[1] * P0_w[1]) <= 0.000001)
    {
        qs[0] = _lastJoint6D.a[0];
        qs[1] = _lastJoint6D.a[0];
        for (i = 0; i < 4; i++)
        {
            _outputSolves.solFlag[0 + i][0] = -1;
            _outputSolves.solFlag[4 + i][0] = -1;
        }
    } else
    {
        qs[0] = atan2f(P0_w[1], P0_w[0]);
        qs[1] = atan2f(-P0_w[1], -P0_w[0]);
        for (i = 0; i < 4; i++)
        {
            _outputSolves.solFlag[0 + i][0] = 1;
            _outputSolves.solFlag[4 + i][0] = 1;
        }
    }
    for (ind_arm = 0; ind_arm < 2; ind_arm++)
    {
        cosqs = cosf(qs[ind_arm] + DH_matrix[0][0]);
        sinqs = sinf(qs[ind_arm] + DH_matrix[0][0]);

        R10[0] = cosqs;
        R10[1] = sinqs;
        R10[2] = 0.0f;
        R10[3] = 0.0f;
        R10[4] = 0.0f;
        R10[5] = -1.0f;
        R10[6] = -sinqs;
        R10[7] = cosqs;
        R10[8] = 0.0f;

        MatMultiply(R10, P0_w, P1_w, 3, 3, 1);
        for (i = 0; i < 3; i++)
        {
            L1_sw[i] = P1_w[i] - L1_base[i];
        }
        l_sw_2 = L1_sw[0] * L1_sw[0] + L1_sw[1] * L1_sw[1];
        l_sw = sqrtf(l_sw_2);

        if (fabs(l_se + l_ew - l_sw) <= 0.000001)
        {
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            qa[0][1] = 0.0f;
            qa[1][1] = 0.0f;
            if (l_sw > l_se + l_ew)
            {
                for (i = 0; i < 2; i++)
                {
                    _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 0;
                    _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 0;
                }
            } else
            {
                for (i = 0; i < 2; i++)
                {
                    _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 1;
                    _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 1;
                }
            }
        } else if (fabs(l_sw - fabs(l_se - l_ew)) <= 0.000001)
        {
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            if (0 == ind_arm)
            {
                qa[0][1] = (float) M_PI;
                qa[1][1] = -(float) M_PI;
            } else
            {
                qa[0][1] = -(float) M_PI;
                qa[1][1] = (float) M_PI;
            }
            if (l_sw < fabs(l_se - l_ew))
            {
                for (i = 0; i < 2; i++)
                {
                    _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 0;
                    _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 0;
                }
            } else
            {
                for (i = 0; i < 2; i++)
                {
                    _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 1;
                    _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 1;
                }
            }
        } else
        {
            atan_a = atan2f(L1_sw[1], L1_sw[0]);
            acos_a = 0.5f * (l_se_2 + l_sw_2 - l_ew_2) / (l_se * l_sw);
            if (acos_a >= 1.0f) acos_a = 0.0f;
            else if (acos_a <= -1.0f) acos_a = (float) M_PI;
            else acos_a = acosf(acos_a);
            acos_e = 0.5f * (l_se_2 + l_ew_2 - l_sw_2) / (l_se * l_ew);
            if (acos_e >= 1.0f) acos_e = 0.0f;
            else if (acos_e <= -1.0f) acos_e = (float) M_PI;
            else acos_e = acosf(acos_e);
            if (0 == ind_arm)
            {
                qa[0][0] = atan_a - acos_a + (float) M_PI_2;
                qa[0][1] = atan_e - acos_e + (float) M_PI;
                qa[1][0] = atan_a + acos_a + (float) M_PI_2;
                qa[1][1] = atan_e + acos_e - (float) M_PI;

            } else
            {
                qa[0][0] = atan_a + acos_a + (float) M_PI_2;
                qa[0][1] = atan_e + acos_e - (float) M_PI;
                qa[1][0] = atan_a - acos_a + (float) M_PI_2;
                qa[1][1] = atan_e - acos_e + (float) M_PI;
            }
            for (i = 0; i < 2; i++)
            {
                _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 1;
                _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 1;
            }
        }
        for (ind_elbow = 0; ind_elbow < 2; ind_elbow++)
        {
            cosqa[0] = cosf(qa[ind_elbow][0] + DH_matrix[1][0]);
            sinqa[0] = sinf(qa[ind_elbow][0] + DH_matrix[1][0]);
            cosqa[1] = cosf(qa[ind_elbow][1] + DH_matrix[2][0]);
            sinqa[1] = sinf(qa[ind_elbow][1] + DH_matrix[2][0]);

            R31[0] = cosqa[0] * cosqa[1] - sinqa[0] * sinqa[1];
            R31[1] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
            R31[2] = 0.0f;
            R31[3] = 0.0f;
            R31[4] = 0.0f;
            R31[5] = 1.0f;
            R31[6] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
            R31[7] = -cosqa[0] * cosqa[1] + sinqa[0] * sinqa[1];
            R31[8] = 0.0f;

            MatMultiply(R31, R10, R30, 3, 3, 3);
            MatMultiply(R30, R06, R36, 3, 3, 3);

            if (R36[8] >= 1.0 - 0.000001)
            {
                cosqw = 1.0f;
                qw[0][1] = 0.0f;
                qw[1][1] = 0.0f;
            } else if (R36[8] <= -1.0 + 0.000001)
            {
                cosqw = -1.0f;
                if (0 == ind_arm)
                {
                    qw[0][1] = (float) M_PI;
                    qw[1][1] = -(float) M_PI;
                } else
                {
                    qw[0][1] = -(float) M_PI;
                    qw[1][1] = (float) M_PI;
                }
            } else
            {
                cosqw = R36[8];
                if (0 == ind_arm)
                {
                    qw[0][1] = acosf(cosqw);
                    qw[1][1] = -acosf(cosqw);
                } else
                {
                    qw[0][1] = -acosf(cosqw);
                    qw[1][1] = acosf(cosqw);
                }
            }
            if (1.0f == cosqw || -1.0f == cosqw)
            {
                if (0 == ind_arm)
                {
                    qw[0][0] = _lastJoint6D.a[3];
                    cosqw = cosf(_lastJoint6D.a[3] + DH_matrix[3][0]);
                    sinqw = sinf(_lastJoint6D.a[3] + DH_matrix[3][0]);
                    qw[0][2] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                    qw[1][2] = _lastJoint6D.a[5];
                    cosqw = cosf(_lastJoint6D.a[5] + DH_matrix[5][0]);
                    sinqw = sinf(_lastJoint6D.a[5] + DH_matrix[5][0]);
                    qw[1][0] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                } else
                {
                    qw[0][2] = _lastJoint6D.a[5];
                    cosqw = cosf(_lastJoint6D.a[5] + DH_matrix[5][0]);
                    sinqw = sinf(_lastJoint6D.a[5] + DH_matrix[5][0]);
                    qw[0][0] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                    qw[1][0] = _lastJoint6D.a[3];
                    cosqw = cosf(_lastJoint6D.a[3] + DH_matrix[3][0]);
                    sinqw = sinf(_lastJoint6D.a[3] + DH_matrix[3][0]);
                    qw[1][2] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                }
                _outputSolves.solFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = -1;
                _outputSolves.solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;
            } else
            {
                if (0 == ind_arm)
                {
                    qw[0][0] = atan2f(R36[5], R36[2]);
                    qw[1][0] = atan2f(-R36[5], -R36[2]);
                    qw[0][2] = atan2f(R36[7], -R36[6]);
                    qw[1][2] = atan2f(-R36[7], R36[6]);
                } else
                {
                    qw[0][0] = atan2f(-R36[5], -R36[2]);
                    qw[1][0] = atan2f(R36[5], R36[2]);
                    qw[0][2] = atan2f(-R36[7], R36[6]);
                    qw[1][2] = atan2f(R36[7], -R36[6]);
                }
                _outputSolves.solFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = 1;
                _outputSolves.solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;
            }
            for (ind_wrist = 0; ind_wrist < 2; ind_wrist++)
            {
                if (qs[ind_arm] > (float) M_PI)
                    _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[0] =
                            qs[ind_arm] - (float) M_PI;
                else if (qs[ind_arm] < -(float) M_PI)
                    _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[0] =
                            qs[ind_arm] + (float) M_PI;
                else
                    _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[0] = qs[ind_arm];

                for (i = 0; i < 2; i++)
                {
                    if (qa[ind_elbow][i] > (float) M_PI)
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[1 + i] =
                                qa[ind_elbow][i] - (float) M_PI;
                    else if (qa[ind_elbow][i] < -(float) M_PI)
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[1 + i] =
                                qa[ind_elbow][i] + (float) M_PI;
                    else
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[1 + i] =
                                qa[ind_elbow][i];
                }

                for (i = 0; i < 3; i++)
                {
                    if (qw[ind_wrist][i] > (float) M_PI)
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[3 + i] =
                                qw[ind_wrist][i] - (float) M_PI;
                    else if (qw[ind_wrist][i] < -(float) M_PI)
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[3 + i] =
                                qw[ind_wrist][i] + (float) M_PI;
                    else
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[3 + i] =
                                qw[ind_wrist][i];
                }
            }
        }
    }

    for (i = 0; i < 8; i++)
        for (float &j: _outputSolves.config[i].a)
            j *= RAD_TO_DEG;

    return true;
}

DOF6Kinematic::Joint6D_t
operator-(const DOF6Kinematic::Joint6D_t &_joints1, const DOF6Kinematic::Joint6D_t &_joints2)
{
    DOF6Kinematic::Joint6D_t tmp{};
    for (int i = 0; i < 6; i++)
        tmp.a[i] = _joints1.a[i] - _joints2.a[i];

    return tmp;
}