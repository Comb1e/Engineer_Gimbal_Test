//
// Created by Henson on 2024-01-14.
//

#ifndef __DRV_DR16_H
#define __DRV_DR16_H

#include "user_lib.h"
#include "usart.h"
#include "freertos_inc.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#define DR16_BUFF_LEN (25)

///* ------------------- RC Switch Definition--------------------- */
//
//#define RC_SW_UP ((uint16_t)1)
//#define RC_SW_MID ((uint16_t)3)
//#define RC_SW_DOWN ((uint16_t)2)

/* ----------------------- RC Event----------------------------- */

#define RC_SW_L_UP2MID   (1 << 0u)//原来为L
#define RC_SW_L_MID2UP   (1 << 1u)
#define RC_SW_L_DOWN2MID (1 << 2u)
#define RC_SW_L_MID2DOWN (1 << 3u)

#define RC_SW_R_UP2MID   (1 << 4u)//原来为R
#define RC_SW_R_MID2UP   (1 << 5u)
#define RC_SW_R_DOWN2MID (1 << 6u)
#define RC_SW_R_MID2DOWN (1 << 7u)

/* ----------------------- RC State ----------------------------- */

#define RC_SW_L_UP       (1 << 8u)
#define RC_SW_L_MID      (1 << 9u)
#define RC_SW_L_DOWN     (1 << 10u)
#define RC_SW_R_UP       (1 << 11u)
#define RC_SW_R_MID      (1 << 12u)
#define RC_SW_R_DOWN     (1 << 13u)


enum keyMap{
    keyW = 0,
    keyS,
    keyA,
    keyD,
    keySHIFT,
    keyCTRL,
    keyQ,
    keyE,
    keyR,
    keyF,
    keyG,
    keyZ,
    keyX,
    keyC,
    keyV,
    keyB,
    keyNum
};

//动作非状态
enum key_dir{
    dir_up = 0,
    dir_down
};

enum RC_SW {
    RC_SW_NONE = 0,
    RC_SW_UP = 1,
    RC_SW_DOWN = 2,
    RC_SW_MID = 3
};

//#pragma pack(1) 告诉编译器以字节对齐数据 要用#pragma pack()恢复设置
#pragma pack(1)
typedef struct {
    uint16_t ch0: 11;//keil中不兼容，待看
    uint16_t ch1: 11;
    uint16_t ch2: 11;
    uint16_t ch3: 11;
    uint8_t s1: 2;
    uint8_t s2: 2;
    int16_t x: 16;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
    uint16_t kb_code;
    uint16_t wheel;
} dr16_raw_info_t;
#pragma pack()

typedef struct{
    float right_x;
    float right_y;
    float left_x;
    float left_y;
} Rocker_t;

typedef struct{
    enum RC_SW sw_l;
    enum RC_SW sw_r;
} switch_t;

typedef struct{
    float x;//移动速度
    float y;
    float z;//z鼠标滚轮
    uint8_t l_click;//状态非动作
    uint8_t r_click;
} mouse_t;

//typedef struct{
//    uint8_t l_click_state : 1;
//    uint8_t r_click_state : 1;
//    uint8_t r_down_event : 1;
//    uint8_t r_up_event : 1;
//    uint8_t l_down_event : 1;
//    uint8_t l_up_event : 1;
//}mouse_bit_t;

typedef struct{
    uint16_t W : 1;
    uint16_t S : 1;
    uint16_t A : 1;
    uint16_t D : 1;
    uint16_t SHIFT : 1;
    uint16_t CTRL : 1;
    uint16_t Q : 1;
    uint16_t E : 1;
    uint16_t R : 1;
    uint16_t F : 1;
    uint16_t G : 1;
    uint16_t Z : 1;
    uint16_t X : 1;
    uint16_t C : 1;
    uint16_t V : 1;
    uint16_t B : 1;
} key_bit_t;

typedef struct {
    Rocker_t rocker;
    switch_t sw;
    mouse_t mouse;
    union {
        uint16_t key_code;
        key_bit_t key_bit;
    } kb;
    float wheel;
} dr16_info_t;

//'沿'状态的判断，普通状态的判断是直接读取data数据
typedef struct{
    uint16_t sw;
    uint16_t keys_up;//只能支持16键
    uint16_t keys_down;//下降边沿
    uint8_t r_click_up;//抬起瞬间
    uint8_t r_click_down;//按下瞬间
    uint8_t l_click_up;
    uint8_t l_click_down;
}event_t;

typedef struct{
    bool is_lost;
    dr16_raw_info_t raw;
    dr16_info_t last_rc_info;//接收
    dr16_info_t rc_info;//接收
    event_t event;
    UART_HandleTypeDef *_huart;
} rc_device_t;


//总函数
void rc_device_init(rc_device_t * rc_dev, UART_HandleTypeDef *_huart, pUART_RxEventCallbackTypeDef rc_pCallback);
bool rc_update(rc_device_t *rc_dev);
void dr16_start_receive_dma(rc_device_t * rc_dev);
void dr16_update(dr16_info_t *dr16_info, uint8_t *buff);
void rc_update_event(rc_device_t *rc_dev);
void rc_update_(rc_device_t *rc_dev, uint8_t *buff);
void set_rc_lost(rc_device_t * rc_dev);
void set_rc_connect(rc_device_t * rc_dev);
bool is_rc_lost(rc_device_t * rc_dev);
void no_rc_protection(rc_device_t * rc_dev);

//状态
bool is_left_click_state(rc_device_t * rc_dev);
bool is_right_click_state(rc_device_t * rc_dev);
bool is_key_down_state(rc_device_t * rc_dev, enum keyMap key);
bool is_key_up_state(rc_device_t * rc_dev, enum keyMap key);
bool is_sw_state(rc_device_t *rc_dev, uint16_t _sw_state);

//事件
bool is_right_click_down_event(rc_device_t * rc_dev);
bool is_right_click_up_event(rc_device_t * rc_dev);
bool is_left_click_down_event(rc_device_t * rc_dev);
bool is_left_click_up_event(rc_device_t * rc_dev);
bool is_key_down_event(rc_device_t * rc_dev, enum keyMap key);
bool is_key_up_event(rc_device_t * rc_dev, enum keyMap key);
bool is_sw_event(rc_device_t *rc_dev, uint16_t _sw_event);

float get_mouse_x();
float get_mouse_y();
float get_mouse_z();
bool is_mouse_left_click_state();
bool is_mouse_right_click_state();


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/




#endif //__DRV_DR16_H
