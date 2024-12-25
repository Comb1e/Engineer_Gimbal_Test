//
// Created by Henson on 2023-10-22.
//

#ifndef __USER_LIB_H
#define __USER_LIB_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef MAX
#define MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifndef MIN
#define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#define high_byte(x) ((uint8_t)(((x) & 0xff00) >> 8))
#define low_byte(x) ((uint8_t)((x) & 0x00ff))
#define merge_bytes(high, low) (((uint16_t)(high) << 8) | (uint16_t)(low))

#define TURE 1
#define FALSE 0

typedef struct{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
}GPIO_t;


typedef enum{
    left,
    right,
    forward,
    backward,
    clockwise,
    anticlockwise,
    directionNum,
}direction_e;

typedef uint8_t lost_num_t;

//这个改变原值但是不返回值
#define VAL_LIMIT(val, min, max) \
  do                             \
  {                              \
    if ((val) <= (min))          \
    {                            \
      (val) = (min);             \
    }                            \
    else if ((val) >= (max))     \
    {                            \
      (val) = (max);             \
    }                            \
  } while (0)


//这个改变原值但是不返回值
#define VAL_RING_LIMIT(val, min, max)   \
    while ((val) < (min))               \
    {                                   \
      (val) = (val)+((max)-(min)+1);    \
    }                                   \
    while ((val) > (max))               \
    {                                   \
      (val) = (val)-((max)-(min)+1);    \
    }

#define ABS_LIMIT(val, limit) \
  do                             \
  {                              \
    if ((val) <= -(limit))          \
    {                            \
      (val) = -(limit);             \
    }                            \
    else if ((val) >= (limit))     \
    {                            \
      (val) = (limit);             \
    }                            \
  } while (0)


#define VAL_DEAD_ZONE(val, min, max) \
  do                             \
  {                              \
    if ((val)>=(min)&&(val)<=(max))  \
    {                            \
      (val) = (0);             \
    }                            \
  } while (0)


#define SIGN(val) ((val)>=0?(1):(-1))


#define ANGLE_LIMIT_360(val, angle) \
  do                                \
  {                                 \
    (val) = (angle) - (int)(angle); \
    (val) += (int)(angle) % 360;    \
  } while (0)

#define ANGLE_LIMIT_180(val, angle) \
  do                                \
  {                                 \
    (val) = (angle) - (int)(angle); \
    (val) += (int)(angle) % 360;    \
    if((val)>180)                   \
      (val) -= 360;                 \
  } while (0)

#define ABS(x) ((x)>=0?(x):-(x))

#define container_of(ptr, type , member) ({ \
	const typeof(((type *)0)->member) *__mptr = (ptr) ; \
	(type *)((char *)__mptr - offsetof(type,member)) ;})

//float abs_zero(float a, float ABS_MAX) {
//    if (a > ABS_MAX) {
//        return 0;
//    }
//    if (a < -ABS_MAX) {
//        return 0;
//    }
//    return a;
//}

#define ABS_ZERO(val,max)  \
  do                        \
  {                         \
    if((val)>=(max))        \
      (val) = 0;             \
    if((val)<=-(max))       \
      (val) = 0              \
  } while (0)


uint32_t get_time_us(void);
uint32_t get_time_ms(void);
float get_time_ms_us(void);
void abs_limit(float *a, float ABS_MAX);
void val_limit(float *a, float VAL_MAX,float VAL_MIN);
float get_min_value(float a, float b);
float get_max_value(float a, float b);
float get_modular(float dividend,float divisor,float *quotient);
bool is_over_time_interval(uint32_t _time_ms);
bool is_under_time_interval(uint32_t _time_ms);
bool is_approx_equal_to(float current_val, float target_val, float _error);
uint16_t unsigned_16(uint8_t *p);
int16_t signed_16(uint8_t *p);
uint32_t unsigned_32(uint8_t *p);
bool is_val_between(float val, float min, float max);

#ifdef __cplusplus
}
#endif

#endif //__USER_LIB_H
