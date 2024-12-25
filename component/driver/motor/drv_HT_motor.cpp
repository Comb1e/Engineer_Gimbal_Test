////
//// Created by Henson on 2024-01-15.
////
//
//#include "drv_HT_motor.h"
//
//uint8_t HT_Motor::get_motor_measure(const uint8_t *data) {
//    if (data[0] == 0 && data[1] == 0 && data[2] == 0) {
//        return 0;
//    } else {
//        motor_id = data[0];
//        angle = uint_to_float(((data[1] << 8) | (data[2])), P_MIN, P_MAX, 16);
//        velocity_watch =  uint_to_float(((data[3] << 4) | (data[4] >> 4)), V_MIN, V_MAX, 12);
//        velocity = uint_to_float(((data[3] << 4) | (data[4] >> 4)), V_MIN, V_MAX, 12)*0.1f+velocity*0.9f;
//        current = (uint_to_float((uint16_t)(((data[4] &0x0f)<< 8) | (data[5])), -18, 18, 12))*1.05f;//((int16_t)((data[4]<<8)|data[5]))&0x0fff;
//
//        position = get_position(95.5f);
//        angle_last = angle;
//        velocity_last = velocity;
//        //帧率监测
//        frequency = 1.f/DWT_GetDeltaT(&motor_cnt);
//        //电机运行监测
//        lost_counter_count_number = 0;
//        return 1;
//    }
//}
//
//void HT_Motor::get_send_data(){
//    /* 限制输入的参数在定义的范围内 */
//    VAL_LIMIT(p,  P_MIN,  P_MAX);
//    VAL_LIMIT(v,  V_MIN,  V_MAX);
//    VAL_LIMIT(kp, KP_MIN, KP_MAX);
//    VAL_LIMIT(kv, KD_MIN, KD_MAX);
//    VAL_LIMIT(torque_ref,  T_MIN,  T_MAX);
//
//    /* 根据协议，对float参数进行转换 */
//    p_temp = float_to_uint(p,   P_MIN,  P_MAX,  16);
//    v_temp = float_to_uint(v,   V_MIN,  V_MAX,  12);
//    kp_temp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
//    kv_temp = float_to_uint(kv, KD_MIN, KD_MAX, 12);
//    torque_temp = float_to_uint(torque_ref,   T_MIN,  T_MAX,  12);
//
//    /* 根据传输协议，把数据转换为CAN命令数据字段 */
//    send_data[0] = p_temp>>8;
//    send_data[1] = p_temp&0xFF;
//    send_data[2] = v_temp>>4;
//    send_data[3] = ((v_temp&0xF)<<4)|(kp_temp>>8);
//    send_data[4] = kp_temp&0xFF;
//    send_data[5] = kv_temp>>4;
//    send_data[6] = ((kv_temp&0xF)<<4)|(torque_temp>>8);
//    send_data[7] = torque_temp&0xff;
//}