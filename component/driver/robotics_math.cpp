//
// Created by Henson on 2024-03-18.
//

#include "robotics_math.h"

#define s arm_sin_f32
#define c arm_cos_f32

inline float cosf(float x) {
    return arm_cos_f32(x);
}

inline float sinf(float x) {
    return arm_sin_f32(x);
}

void MatMultiply(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _l, const int _n)
{
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            tmp = 0.0f;
            for (k = 0; k < _l; k++)
            {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}

void zyxEulerAngleToRotMat(const float* _eulerAngles, float* _rotationM)
{
    float ca, cb, cc, sa, sb, sc;

    cc = cosf(_eulerAngles[0]);
    cb = cosf(_eulerAngles[1]);
    ca = cosf(_eulerAngles[2]);
    sc = sinf(_eulerAngles[0]);
    sb = sinf(_eulerAngles[1]);
    sa = sinf(_eulerAngles[2]);

    _rotationM[0] = ca * cb;
    _rotationM[1] = ca * sb * sc - sa * cc;
    _rotationM[2] = ca * sb * cc + sa * sc;
    _rotationM[3] = sa * cb;
    _rotationM[4] = sa * sb * sc + ca * cc;
    _rotationM[5] = sa * sb * cc - ca * sc;
    _rotationM[6] = -sb;
    _rotationM[7] = cb * sc;
    _rotationM[8] = cb * cc;
}

//xyx旋转矩阵变换
void euler_xyx_to_rotMatrix(float x1, float y, float x2, float mat[9]){
    //第一行
    mat[0] = c(y);
    mat[1] = s(y) * s(x2);
    mat[2] = s(y) * c(x2);

    //第二行
    mat[3] = s(x1) * s(y);
    mat[4] = -s(x1) * c(y) * s(x2) + c(x1) * c(x2);
    mat[5] = -s(x1) * c(y) * c(x2) - c(x1) * s(x2);

    //第三行
    mat[6] = -c(x1) * s(y);
    mat[7] = c(x1) * c(y) * s(x2) + s(x1) * c(x2);
    mat[8] = c(x1) * c(y) * c(x2) - s(x1) * s(x2);
}

//XYX坐标系变换
//矩阵按行排列
Eigen::Vector3f rotMatrix_to_euler_xyx_pos(float matrix[9])
{
    Eigen::Vector3f euler_xyx;
    float sin_y,_y = 0.0f;
    arm_sqrt_f32(matrix[1]*matrix[1]+matrix[2]*matrix[2],&sin_y);
    arm_atan2_f32(sin_y,matrix[0],&_y);//取正号 y有范围为[0,180]
    if(abs(_y)<1e-6){
        euler_xyx(0) = 0;
        euler_xyx(1) = 0;
        arm_atan2_f32(matrix[7],matrix[8],&euler_xyx(2));
    }else if(abs(_y-PI)<1e-6){
        euler_xyx(0) = 0;
        euler_xyx(1) = PI;
        arm_atan2_f32(-matrix[7],-matrix[8],&euler_xyx(2));
    }else{  //x1取值-PI ~ PI,所以有限幅的时候得+—
        arm_atan2_f32(matrix[3]/sin_y,-matrix[6]/sin_y,&euler_xyx(0));
        arm_atan2_f32(matrix[1]/sin_y,matrix[2]/sin_y,&euler_xyx(2));
        euler_xyx(1) = _y;
    }
    return euler_xyx;

//    float alpha = 0.0f, beta = 0.0f ,gamma = 0.0f;
//    Eigen::Vector3f euler_xyx;
//    euler_xyx << 0,0,0;
//
//    beta = (float)std::acos(matrix[0]);//[0-PI] 可以取负 对于y
//
//    if(ABS(beta)<1e-6){//beta = 0
//        alpha = 0.0f;//一般情况下
//        beta = 0.0f;
//        arm_atan2_f32(matrix[7],matrix[8],&gamma);//(-PI,PI]
//    }
//    else if(ABS(beta)-PI<1e-5){
//        alpha = 0.0f;//一般情况下
//        beta = PI;//或者-PI
//        arm_atan2_f32(-matrix[7],-matrix[8],&gamma);
//    }
//    else{
//        arm_atan2_f32(matrix[3],-matrix[6],&alpha);//(-PI,PI]
//        arm_atan2_f32(matrix[1],matrix[2],&gamma);//(-PI,PI]
//    }
//
//    __NOP();
//    euler_xyx << alpha, beta, gamma;
//    return euler_xyx;
}


//XYX坐标系变换
//矩阵按行排列
Eigen::Vector3f rotMatrix_to_euler_xyx_neg(float matrix[9])
{
    Eigen::Vector3f euler_xyx;
    float sin_y, _y = 0.0f;
    arm_sqrt_f32(matrix[1] * matrix[1] + matrix[2] * matrix[2], &sin_y);
    sin_y = -sin_y;
    arm_atan2_f32(sin_y, matrix[0], &_y);//取负号 y有范围为[-180,0]
    if (abs(_y) < 1e-6) {
        euler_xyx(0) = 0;
        euler_xyx(1) = 0;
        arm_atan2_f32(matrix[7], matrix[8], &euler_xyx(2));
    } else if (abs(_y - PI) < 1e-6) {
        euler_xyx(0) = 0;
        euler_xyx(1) = PI;
        arm_atan2_f32(-matrix[7], -matrix[8], &euler_xyx(2));
    } else {  //x1取值-PI ~ PI,所以有限幅的时候得+—
        arm_atan2_f32(matrix[3] / sin_y, -matrix[6] / sin_y, &euler_xyx(0));
        arm_atan2_f32(matrix[1] / sin_y, matrix[2] / sin_y, &euler_xyx(2));
        euler_xyx(1) = _y;
    }
    return euler_xyx;

}

//ZYX坐标系变换 //默认y取[-180,180]
//矩阵按行排列
Eigen::Vector3f rotMatrix_to_euler_zyx(float matrix[9]) {
    float alpha = 0.0f, beta = 0.0f ,gamma = 0.0f;
    Eigen::Vector3f euler_zyx;
    euler_zyx << 0,0,0;

    beta = asin(-matrix[6]);
    if(abs(beta - PI/2) < 1e-6){
        alpha = 0;
        beta = PI/2;
        arm_atan2_f32(matrix[1],matrix[2],&gamma);
    }
    else{
        arm_atan2_f32(matrix[3],matrix[0],&alpha);
        arm_atan2_f32(matrix[7],matrix[8],&gamma);
    }

    euler_zyx << alpha, beta, gamma;
    return euler_zyx;
}

void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles) {
    float A, B, C, cb;

    if (fabs(_rotationM[6]) >= 1.0 - 0.0001) {
        if (_rotationM[6] < 0) {
            A = 0.0f;
            B = (float) M_PI_2;
            C = atan2f(_rotationM[1], _rotationM[4]);
        } else {
            A = 0.0f;
            B = -(float) M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    } else {
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        cb = cosf(B);
        A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }

    _eulerAngles[0] = C;
    _eulerAngles[1] = B;
    _eulerAngles[2] = A;
}

#undef s
#undef c
