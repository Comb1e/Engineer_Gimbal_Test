//
// Created by Henson on 2024-01-14.
//

#include "drv_dr16.h"

uint8_t dr16_buff[DR16_BUFF_LEN] = {0};
rc_device_t g_rc={.is_lost = true};
extern lost_num_t g_lost_num;

void rc_device_init(rc_device_t * rc_dev, UART_HandleTypeDef *_huart, pUART_RxEventCallbackTypeDef rc_pCallback){
    rc_dev->is_lost = true;//保证不发送数据
    rc_dev->_huart = _huart;
    HAL_UART_RegisterRxEventCallback(g_rc._huart, rc_pCallback);
}

void dr16_start_receive_dma(rc_device_t * rc_dev){
    HAL_UARTEx_ReceiveToIdle_DMA(rc_dev->_huart, (uint8_t *)&rc_dev->raw, DR16_BUFF_LEN);//封装好的函数
    __HAL_DMA_DISABLE_IT(rc_dev->_huart->hdmarx, DMA_IT_HT);//关闭半满中断
    //__HAL_DMA_DISABLE_IT(rc_dev->_huart->hdmarx,DMA_IT_TC);//关闭全满中断
}

void dr16_update(dr16_info_t *dr16_info, uint8_t *buff) {
    dr16_raw_info_t *dr16_raw_info = (dr16_raw_info_t *) buff;//这个地方没用联合体，用的强转
    int16_t ch[4];
    ch[0] = (int16_t) (dr16_raw_info->ch0 - 1024);
    ch[1] = (int16_t) (dr16_raw_info->ch1 - 1024);
    ch[2] = (int16_t) (dr16_raw_info->ch2 - 1024);
    ch[3] = (int16_t) (dr16_raw_info->ch3 - 1024);
    for (int i = 0; i < 4; i++) {
        if (ch[i] < -660 || ch[i] > 660) {
            return;
        }
    }
    for (int i = 0; i < 4; i++) {
        if (ch[i] < 12 && ch[i] > -12) {
            ch[i] = 0;
        }
    }

    dr16_info->rocker.right_x = (float)ch[0] / 660.0f;
    dr16_info->rocker.right_y = (float)ch[1] / 660.0f;
    dr16_info->rocker.left_x = (float)ch[2] / 660.0f;
    dr16_info->rocker.left_y = (float)ch[3] / 660.0f;

    dr16_info->sw.sw_l = (enum RC_SW)dr16_raw_info->s2;//遥控器反了?
    dr16_info->sw.sw_r = (enum RC_SW)dr16_raw_info->s1;

    // 32768.0f;//速度
    dr16_info->mouse.x = ((float)dr16_raw_info->x /500.0f)>1.0f? 1.0f:((float)dr16_raw_info->x /500.0f)<-1.0f? -1.0f:(float)dr16_raw_info->x /500.0f;
    dr16_info->mouse.y = ((float)dr16_raw_info->y /500.0f)>1.0f? 1.0f:((float)dr16_raw_info->y /500.0f)<-1.0f? -1.0f:(float)dr16_raw_info->y /500.0f;
    dr16_info->mouse.z = ((float)dr16_raw_info->z /40.0f)>1.0f? 1.0f:((float)dr16_raw_info->z /40.0f)<-1.0f? -1.0f:(float)dr16_raw_info->z /40.0f;
    //0.25-0.50-0.75-1.00

    dr16_info->mouse.l_click = dr16_raw_info->press_l;
    dr16_info->mouse.r_click = dr16_raw_info->press_r;

    dr16_info->kb.key_code = dr16_raw_info->kb_code;
    dr16_info->wheel = -(float) (dr16_raw_info->wheel - 1024) / 660.0f;
}


// RC_SW_UP 1
// RC_SW_MID 3
// RC_SW_DOWN 2
void rc_update_event(rc_device_t *rc_dev) {
    //位
    rc_dev->event.keys_down = ((~rc_dev->last_rc_info.kb.key_code)&(rc_dev->rc_info.kb.key_code));
    rc_dev->event.keys_up   = ((rc_dev->last_rc_info.kb.key_code)&(~rc_dev->rc_info.kb.key_code));
    //逻辑
    rc_dev->event.r_click_down = rc_dev->rc_info.mouse.r_click && (!rc_dev->last_rc_info.mouse.r_click);
    rc_dev->event.r_click_up = (!rc_dev->rc_info.mouse.r_click) && rc_dev->last_rc_info.mouse.r_click;
    rc_dev->event.l_click_down = rc_dev->rc_info.mouse.l_click && (!rc_dev->last_rc_info.mouse.l_click);
    rc_dev->event.l_click_up = (!rc_dev->rc_info.mouse.l_click) && rc_dev->last_rc_info.mouse.l_click;

    if (rc_dev->rc_info.sw.sw_l == RC_SW_MID) {
        if (rc_dev->last_rc_info.sw.sw_l == RC_SW_UP) {
            rc_dev->event.sw &= ~0x0F;
            rc_dev->event.sw |= RC_SW_L_UP2MID;
        } else if (rc_dev->last_rc_info.sw.sw_l == RC_SW_DOWN) {
            rc_dev->event.sw &= ~0x0F;
            rc_dev->event.sw |= RC_SW_L_DOWN2MID;
        }
    } else if (rc_dev->rc_info.sw.sw_l == RC_SW_UP) {
        if (rc_dev->last_rc_info.sw.sw_l == RC_SW_MID) {
            rc_dev->event.sw &= ~0x0F;
            rc_dev->event.sw |= RC_SW_L_MID2UP;
        }
    } else if (rc_dev->rc_info.sw.sw_l == RC_SW_DOWN) {
        if (rc_dev->last_rc_info.sw.sw_l == RC_SW_MID) {
            rc_dev->event.sw &= ~0x0F;
            rc_dev->event.sw |= RC_SW_L_MID2DOWN;
        }
    }

    if (rc_dev->rc_info.sw.sw_r == RC_SW_MID) {
        if (rc_dev->last_rc_info.sw.sw_r == RC_SW_UP) {
            rc_dev->event.sw &= ~0xF0;
            rc_dev->event.sw |= RC_SW_R_UP2MID;
        } else if (rc_dev->last_rc_info.sw.sw_r == RC_SW_DOWN) {
            rc_dev->event.sw &= ~0xF0;
            rc_dev->event.sw |= RC_SW_R_DOWN2MID;
        }
    } else if (rc_dev->rc_info.sw.sw_r == RC_SW_UP) {
        if (rc_dev->last_rc_info.sw.sw_r == RC_SW_MID) {
            rc_dev->event.sw &= ~0xF0;
            rc_dev->event.sw |= RC_SW_R_MID2UP;
        }
    } else if (rc_dev->rc_info.sw.sw_r == RC_SW_DOWN) {
        if (rc_dev->last_rc_info.sw.sw_r == RC_SW_MID) {
            rc_dev->event.sw &= ~0xF0;
            rc_dev->event.sw |= RC_SW_R_MID2DOWN;
        }
    }
}

void rc_update_(rc_device_t *rc_dev, uint8_t *buff) {
    memcpy(&rc_dev->last_rc_info, &rc_dev->rc_info, sizeof(rc_dev->rc_info));
    dr16_update(&rc_dev->rc_info, buff);
    rc_update_event(rc_dev);
}

//总函数
bool rc_update(rc_device_t *_rc_dev) {
    if (DR16_BUFF_LEN - __HAL_DMA_GET_COUNTER(_rc_dev->_huart->hdmarx) == 18) {
        rc_update_(_rc_dev, (uint8_t *)&_rc_dev->raw);
        return true;
    } else {
        return false;
    }
}

float get_mouse_x(){
    return g_rc.rc_info.mouse.x;
}
float get_mouse_y(){
    return g_rc.rc_info.mouse.y;
}
float get_mouse_z(){
    return g_rc.rc_info.mouse.z;
}

bool is_mouse_left_click_state(){
    return g_rc.rc_info.mouse.l_click;
}

bool is_mouse_right_click_state(){
    return g_rc.rc_info.mouse.r_click;
}

void set_rc_connect(rc_device_t * rc_dev){
    rc_dev->is_lost = false;
}

void set_rc_lost(rc_device_t * rc_dev){
    rc_dev->is_lost = true;
}


bool is_rc_lost(rc_device_t * rc_dev){
    return rc_dev->is_lost;
}

//读数据是读状态,读event是读变化
bool is_right_click_state(rc_device_t * rc_dev){
    return (bool)rc_dev->rc_info.mouse.r_click;
}
bool is_left_click_state(rc_device_t * rc_dev){
    return (bool)rc_dev->rc_info.mouse.l_click;
}
bool is_key_down_state(rc_device_t * rc_dev, enum keyMap key) {
    switch(key){
        case keyW:return rc_dev->rc_info.kb.key_bit.W;
            break;
        case keyS:return rc_dev->rc_info.kb.key_bit.S;
            break;
        case keyA:return rc_dev->rc_info.kb.key_bit.A;
            break;
        case keyD:return rc_dev->rc_info.kb.key_bit.D;
            break;
        case keySHIFT:return rc_dev->rc_info.kb.key_bit.SHIFT;
            break;
        case keyCTRL:return rc_dev->rc_info.kb.key_bit.CTRL;
            break;
        case keyQ:return rc_dev->rc_info.kb.key_bit.Q;
            break;
        case keyE:return rc_dev->rc_info.kb.key_bit.E;
            break;
        case keyR:return rc_dev->rc_info.kb.key_bit.R;
            break;
        case keyF:return rc_dev->rc_info.kb.key_bit.F;
            break;
        case keyG:return rc_dev->rc_info.kb.key_bit.G;
            break;
        case keyZ:return rc_dev->rc_info.kb.key_bit.Z;
            break;
        case keyX:return rc_dev->rc_info.kb.key_bit.X;
            break;
        case keyC:return rc_dev->rc_info.kb.key_bit.C;
            break;
        case keyV:return rc_dev->rc_info.kb.key_bit.V;
            break;
        case keyB:return rc_dev->rc_info.kb.key_bit.B;
            break;
        default:return 0;
    }
}
bool is_key_up_state(rc_device_t * rc_dev, enum keyMap key) {
    return !is_key_down_state(rc_dev,key);
}

//读event是读变化并且检测到后就清除,读数据是读状态,不用清除
bool is_sw_state(rc_device_t *rc_dev, uint16_t _sw_state) {

    if (rc_dev->rc_info.sw.sw_l == RC_SW_MID && _sw_state==RC_SW_L_MID) {
        return true;
    }
    else if (rc_dev->rc_info.sw.sw_l == RC_SW_UP && _sw_state==RC_SW_L_UP) {
        return true;
    }
    else if (rc_dev->rc_info.sw.sw_l == RC_SW_DOWN && _sw_state==RC_SW_L_DOWN) {
        return true;
    }
    else if (rc_dev->rc_info.sw.sw_r == RC_SW_MID && _sw_state==RC_SW_R_MID) {
        return true;
    }
    else if (rc_dev->rc_info.sw.sw_r == RC_SW_UP && _sw_state==RC_SW_R_UP) {
        return true;
    }
    else if (rc_dev->rc_info.sw.sw_r == RC_SW_DOWN && _sw_state==RC_SW_R_DOWN) {
        return true;
    }

    return false;
}


//读数据是读状态,读event是读变化,检测到后就清除
bool is_sw_event(rc_device_t *rc_dev, uint16_t _sw_event) {
    taskENTER_CRITICAL();
    if ((rc_dev->event.sw & _sw_event) == _sw_event) {
        rc_dev->event.sw &= (~(_sw_event & 0x00FF));
        taskEXIT_CRITICAL();
        return true;
    } else {
        taskEXIT_CRITICAL();
        return false;
    }
}

bool is_right_click_down_event(rc_device_t * rc_dev){
    taskENTER_CRITICAL();
    if(rc_dev->event.r_click_down){
        rc_dev->event.r_click_down = 0;
        taskEXIT_CRITICAL();
        return true;
    }
    else{
        taskEXIT_CRITICAL();
        return false;
    }
}
bool is_left_click_down_event(rc_device_t * rc_dev){
    taskENTER_CRITICAL();
    if(rc_dev->event.l_click_down){
        rc_dev->event.l_click_down = 0;
        taskEXIT_CRITICAL();
        return true;
    }
    else{
        taskEXIT_CRITICAL();
        return false;
    }
}
bool is_right_click_up_event(rc_device_t * rc_dev){
    taskENTER_CRITICAL();
    if(rc_dev->event.r_click_up){
        rc_dev->event.r_click_up = 0;
        taskEXIT_CRITICAL();
        return true;
    }
    else{
        taskEXIT_CRITICAL();
        return false;
    }
}
bool is_left_click_up_event(rc_device_t * rc_dev){
    taskENTER_CRITICAL();
    if(rc_dev->event.l_click_up){
        rc_dev->event.l_click_up = 0;
        taskEXIT_CRITICAL();
        return true;
    }
    else {
        taskEXIT_CRITICAL();
        return false;
    }
}


bool is_key_down_event(rc_device_t * rc_dev, enum keyMap key){
    uint16_t key_state = (1<<key);
    taskENTER_CRITICAL();
    if ((rc_dev->event.keys_down & key_state) == key_state)
    {
        rc_dev->event.keys_down &= (~key_state);
        taskEXIT_CRITICAL();
        return true;
    }
    else{
        taskEXIT_CRITICAL();
        return false;
    }
};

bool is_key_up_event(rc_device_t * rc_dev, enum keyMap key){
    uint16_t key_state = (1<<key);
    taskENTER_CRITICAL();
    if ((rc_dev->event.keys_up & key_state) == key_state)
    {
        rc_dev->event.keys_up &= (~key_state);
        taskEXIT_CRITICAL();
        return true;
    }
    else{
        taskEXIT_CRITICAL();
        return false;
    }
}

