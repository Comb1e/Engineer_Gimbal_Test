//
// Created by Henson on 2024-01-13.
//

#ifndef __MIDDLEWARE_BMI088_H
#define __MIDDLEWARE_BMI088_H

#include "main.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

extern SPI_HandleTypeDef *BMI088_SPI;

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__MIDDLEWARE_BMI088_H
