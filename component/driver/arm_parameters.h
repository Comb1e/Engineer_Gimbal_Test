//
// Created by Henson on 2024-03-21.
//

#ifndef __ARM_PARAMETERS_H
#define __ARM_PARAMETERS_H

/*---------------------------- C++ Scope ----------------------------*/
//三平面确定原点法  z=0平面 交 x=0平面 交 y=0平面
/*---------------------------- arm-parameters ---------------------------*/

//arm-parameters
#define ARM_YAW_2_PITCH_LEN_X_MM    (200.0f)
#define ARM_YAW_2_PITCH_LEN_Z_MM    (0.0f)
#define ARM_PITCH_2_X1_LEN_X_MM     (60.0f)
#define X1_2_YX2_LEN_X_MM           (135.0f)
#define YX2_LEN_X_MM                (0.0f)

//calculate
#define X1_DEDUCTION_RATIO          (2.0f/5.0f)
#define SMALL_BEVEL_GEAR            (15.0f)
#define BIG_BEVEL_GEAR              (40.0f)
#define SMALL_BELT_GEAR             (24.0f) //20 或 24  25或24
#define Y_X2_BELT_GEAR              (20.0f)
#define MAX_VELOCITY_MATCH_RATIO    (((SMALL_BELT_GEAR / Y_X2_BELT_GEAR) * (1.0f/(SMALL_BEVEL_GEAR/BIG_BEVEL_GEAR)))/((1.0f / X1_DEDUCTION_RATIO)))

/*---------------------------- framework ---------------------------*/ //只对每个电机单纯限幅就可以了

//x 向后复位
#define FRAME_EXTEND_MIN_MM             0
#define FRAME_EXTEND_MAX_MM             660

#define FRAME_EXTEND_L_MIN_ROUNDS       0
#define FRAME_EXTEND_R_MIN_ROUNDS       0
#define FRAME_EXTEND_L_MIN_ROUNDS_OFFSET  (-0.9f)
#define FRAME_EXTEND_R_MIN_ROUNDS_OFFSET  (-0.9f)

#define FRAME_EXTEND_L_MAX_ROUNDS       80.2f
#define FRAME_EXTEND_R_MAX_ROUNDS       80.2f

//y  向左复位
#define FRAME_SLIDE_MIN_MM              0
#define FRAME_SLIDE_MAX_MM              138.0f

#define FRAME_SLIDE_MIN_ROUNDS          0
#define FRAME_SLIDE_MAX_ROUNDS          65.2f
#define FRAME_SLIDE_MAX_ROUNDS_OFFSET   (0.3f)

//z 向下复位
#define FRAME_UPLIFT_MIN_MM             (0.0f)
#define FRAME_UPLIFT_MAX_MM             (660.0f)//640-660
//从初始位置到地320

#define FRAME_UPLIFT_L_MIN_ROUNDS       (0.0f)
#define FRAME_UPLIFT_R_MIN_ROUNDS       (0.0f)

#define FRAME_UPLIFT_L_MIN_ROUNDS_OFFSET  (-0.22f)
#define FRAME_UPLIFT_R_MIN_ROUNDS_OFFSET  (-0.22f)


#define FRAME_UPLIFT_L_MAX_ROUNDS       (48.38f)//48.46
#define FRAME_UPLIFT_R_MAX_ROUNDS       (48.38f)//48.46

#define FRAME_UPLIFT_OFFSET_TORQUE      (0.22f)

//arm yaw  向左复位
#define ARM_YAW_MIN_ROUNDS              (-60.0f/360.0f)
#define ARM_YAW_MAX_ROUNDS              (60.0f/360.0f)

#define ARM_YAW_MIN_DEG                 (ARM_YAW_MIN_ROUNDS*360.0f)
#define ARM_YAW_MAX_DEG                 (ARM_YAW_MAX_ROUNDS*360.0f)

#define ARM_YAW_ZERO_OFFSET_ROUNDS      (0.4956f)

//arm pitch  向左复位
#define ARM_PITCH_MIN_ROUNDS            (-3.8f/360.0f)
#define ARM_PITCH_MAX_ROUNDS            (90.0f/360.0f)

#define ARM_PITCH_MIN_DEG               (ARM_PITCH_MIN_ROUNDS*360.0f)
#define ARM_PITCH_MAX_DEG               (ARM_PITCH_MAX_ROUNDS*360.0f)

#define ARM_PITCH_ZERO_OFFSET_ROUNDS    (0.3303f)

/*---------------------------- actuator ---------------------------*/  //只对每个电机单纯限幅就可以了
//actuator
#define ACTUATOR_X1_MIN_DEG         (-1080.0f)
#define ACTUATOR_X1_MAX_DEG         (1080.0f)
#define HORIZONTAL_ARM_ROLL_DEG     (0.0f)

#define ACTUATOR_Y_MIN_DEG          (-90.0f)
#define ACTUATOR_Y_MAX_DEG          (90.0f)

#define ACTUATOR_X2_MIN_DEG         (-1800.0f)
#define ACTUATOR_X2_MAX_DEG         (1800.0f)//千万别和min一样

//各减去0.12代表1.0° 越大则越朝上
#define Y_X2_U_OFFSET_ROUNDS        (-15.0f)
#define Y_X2_D_OFFSET_ROUNDS        (-15.0f)

/*---------------------------- measure in real world ---------------------------*/

#define YAW_DEG_MIN     (-160.0f)
#define YAW_DEG_MAX     160

#define PITCH_DEG_MIN   (-100.0f)
#define PITCH_DEG_MAX   100

#define ROLL_DEG_MIN    (-360.0f)
#define ROLL_DEG_MAX    360

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif //__ARM_PARAMETERS_H
