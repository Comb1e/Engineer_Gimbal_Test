//
// Created by Henson on 2024-03-14.
//

#ifndef __DRV_DOF_H
#define __DRV_DOF_H

/*---------------------------- C++ Scope ---------------------------*/
typedef struct{
    float x_mm;
    float y_mm;
    float z_mm;
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
}dof6_t;

typedef enum {
    x = 1,
    y = 2,
    z = 3,
    yaw = 4,
    pitch = 5,
    roll = 6
}dof6_e;

typedef struct{
    float x_mm;
    float y_mm;
    float z_mm;
}dof3_t;

typedef enum {
    x_3 = 1,
    y_3 = 2,
    z_3 = 3,
}dof3_e;

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif //__DRV_DOF_H
