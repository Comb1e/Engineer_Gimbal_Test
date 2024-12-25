//
// Created by Henson on 2024-03-18.
//

#ifndef __ROBOTICS_MATH_H
#define __ROBOTICS_MATH_H

/*---------------------------- C++ Scope ---------------------------*/
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "arm_math.h"
#include "user_lib.h"


Eigen::Vector3f rotMatrix_to_euler_zyx(float matrix[9]);
Eigen::Vector3f rotMatrix_to_euler_xyx_pos(float matrix[9]);
Eigen::Vector3f rotMatrix_to_euler_xyx_neg(float matrix[9]);
void euler_xyx_to_rotMatrix(float x1, float y, float x2, float mat[9]);
void MatMultiply(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                 const int _m, const int _l, const int _n);
void zyxEulerAngleToRotMat(const float* _eulerAngles, float* _rotationM);
void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles);


/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif //__ROBOTICS_MATH_H
