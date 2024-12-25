////
//// Created by Henson on 2024-03-07.
////
//
//#ifndef __DRV_HIWONDER_MOTOR_H
//#define __DRV_HIWONDER_MOTOR_H
//
//#include "drv_motor.h"
//#include "bsp_usart.h"
//
///** @defgroup Instructions */
//
//#define LOBOT_SERVO_FRAME_HEADER         0x55
//#define LOBOT_SERVO_MOVE_TIME_WRITE      1
//#define LOBOT_SERVO_MOVE_TIME_READ       2
//#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
//#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
//#define LOBOT_SERVO_MOVE_START           11
//#define LOBOT_SERVO_MOVE_STOP            12
//#define LOBOT_SERVO_ID_WRITE             13
//#define LOBOT_SERVO_ID_READ              14
//#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
//#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
//#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
//#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
//#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
//#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
//#define LOBOT_SERVO_VIN_LIMIT_READ       23
//#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
//#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
//#define LOBOT_SERVO_TEMP_READ            26
//#define LOBOT_SERVO_VIN_READ             27
//#define LOBOT_SERVO_POS_READ             28
//#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
//#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
//#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
//#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
//#define LOBOT_SERVO_LED_CTRL_WRITE       33
//#define LOBOT_SERVO_LED_CTRL_READ        34
//#define LOBOT_SERVO_LED_ERROR_WRITE      35
//#define LOBOT_SERVO_LED_ERROR_READ       36
//
///** @defgroup ErrorCode */
//#define LOBOT_ERROR_NO                   0
//#define LOBOT_ERROR_TEMPERATURE          1
//#define LOBOT_ERROR_PRESSURE             2
//#define LOBOT_ERROR_TEMP_PRES            3
//#define LOBOT_ERROR_BLOCK                4
//#define LOBOT_ERROR_TEMP_BLOCK           5
//#define LOBOT_ERROR_PRES_BLOCK           6
//#define LOBOT_ERROR_TEMP_PRES_BLO        7
//
///** @defgroup Mode */
//#define LOBOT_SERVO_MODE_SERVO           0
//#define LOBOT_SERVO_MODE_ENGINE          1
//
//#define UINT8_ERROR_RETURN              (uint8_t)255
//#define INT8_ERROR_RETURN               (int8_t)127
//#define UINT16_ERROR_RETURN             (uint16_t)65535
//#define INT16_ERROR_RETURN              (int16_t)32767
//
//#define LOBOT_BROADCAST_ID              0xFE   //254
//
//
///*--------------------------- user define ---------------------------*/
//#define LOBOT_SERVO_ANG_MAX             240.0f
//#define LOBOT_SERVO_POS_MAX             1000.0f
//#define LOBOT_ENGINE_SPEED_MAX          1000.0f
//#define LOBOT_SERVO_POS2ANG             0.24f
//#define LOBOT_SERVO_ANG2POS             4.166666666667f
//
//#define SERVO_RX_BUFF_LEN               16
//#define LOBOT_SERVO_NUM                 2
///*---------------------------- C++ Scope ---------------------------*/
//class Hiwonder_Motor:public Motor{
//public:
//    typedef struct{
//        uint8_t id;
//        bool is_load;
//        bool is_servo;
//        bool is_engine;
//        uint8_t error_code;
//
//        struct{//没有考虑到stop      //这个是用来设置的,不是根据命令来改变的,或者单独来写一个set的结构体也可以
//            bool is_need_start;
//            int8_t pos_offset;
//            uint16_t pos_max;
//            uint16_t pos_min;
//            int16_t current_pos;//28命令读出可能为负，写入则为正
//            uint16_t pre_pos;
//
//            float ang_offset;
//            float ang_max;
//            float ang_min;
//            float current_ang;
//            float pre_ang;
//        }servo_mode;
//
//        struct{
//            bool is_reverse;
//            float speedf;
//        }engine_mode;
//    }servo_device_t;
//
//    UART_HandleTypeDef *huart;
//
//    float max_torque{};
//    float max_speed{};
//    float max_ecd{};
//
//    uint32_t stall_cnt{};
//    int16_t stall_torque_max = 6000; // torque && speed
//    float stall_speed_max = 0.03f; // torque && speed
//
//
//    bool is_enable = false;
//    servo_device_t servo_dev[LOBOT_SERVO_NUM]{};
//    servo_device_t *id2dev[254]{};//设备id范围为0-253
//
//public:
////parent
//    void set_motor_reverse(uint8_t _id){
//        id2dev[_id]->engine_mode.is_reverse = true;
//    }
//
//    void set_motor_forward(uint8_t _id){
//        id2dev[_id]->engine_mode.is_reverse = false;
//    }
//
//    void set_motor_toggle(uint8_t _id){
//        id2dev[_id]->engine_mode.is_reverse = !id2dev[_id]->engine_mode.is_reverse;
//    }
//
//    float get_motor_speed();
//    float get_motor_total_rounds();
//    float get_motor_offset_round();
//    float get_motor_current_round();
//
//
//    void set_motor_offset(float _offset_round);
//    void set_motor_total_rounds_offset(float _total_rounds);
//
////now
//    explicit Hiwonder_Motor(UART_HandleTypeDef *huart);
//
//    void set_motor_stall_parameter(int16_t _stall_torque_max,float _stall_speed_max);
//    void motor_data_update(uint8_t *can_rx_data);
//
//    bool is_motor_stall(float _critical_speed, uint16_t _critical_current);
//    void set_motor_speed(float speed) override;
//    void set_motor_rounds(float rounds) override;
//    bool set_motor_reset_position(float reset_speed) override;
//
//
//    void set_motor_free(uint8_t _id);
//    int16_t get_fb_torque();
//    int8_t get_temperature(uint8_t _id);
//
//
//    bool LobotSerialDeviceAdd(uint8_t _dev_index, uint8_t _id);
//
//    void LobotSerialServoSetPosf(uint8_t _id, float _position_f, uint16_t _time_ms);
//    void LobotSerialServoSetAng(uint8_t _id, float _ang_f, uint16_t _time_ms);
//    void LobotSerialServoSetEngineSpeedf(uint8_t _id, float _speed_f);
//    void LobotSerialServoAngLimit(uint8_t _id, float _ang_min_f, float _ang_max_f);
//    void LobotSerialServoChangeAngOffset(uint8_t _id, float  _offset_ang_f);
//    void LobotSerialServoSetSaveAngOffset(uint8_t _id, float _offset_ang_f);
//    void LobotSerialServoSetSavePosOffset(uint8_t _id, int8_t _offset_pos);
//    uint8_t LobotSerialReadID();
//
//
////id可设置0-253
//    //write
//    void LobotSerialServoMoveSet(uint8_t _id, int16_t _position, uint16_t _time_ms);
//    void LobotSerialServoPreMoveSet(uint8_t _id, int16_t _pre_pos, uint16_t _pre_time_ms);
//    void LobotSerialServoStart(uint8_t _id);
//    void LobotSerialServoStop(uint8_t _id);
//    void LobotSerialServoSetID(uint8_t oldID, uint8_t newID);
//    void LobotSerialServoChangePosOffset(uint8_t _id, int8_t _offset_pos);
//    void LobotSerialServoSaveOffset(uint8_t _id);
//    void LobotSerialServoPosLimit(uint8_t _id, uint16_t _pos_min, uint16_t _pos_max);
//    void LobotSerialServoVoltageLimit(uint8_t _id, uint16_t _vol_min, uint16_t _vol_max);
//    void LobotSerialServoTempLimit(uint8_t _id, uint8_t _temp_max);
//    void LobotSerialServoSetEngineMode(uint8_t _id, int16_t _speed);
//    void LobotSerialServoSetServoMode(uint8_t _id);
//    void LobotSerialServoSetUnload(uint8_t _id);
//    void LobotSerialServoSetLoad(uint8_t _id);
//    void LobotSerialServoErrorCfg(uint8_t _id, uint8_t _error_type);
//
//    //read
//    uint8_t LobotSerialServoReadID(uint8_t _id);
//    int8_t LobotSerialServoReadPosOffset(uint8_t _id);
//    uint8_t LobotSerialServoReadTemp(uint8_t _id);
//    int16_t LobotSerialServoReadVoltage(uint8_t _id);
//    int16_t LobotSerialServoReadPosition(uint8_t _id);
//    uint8_t LobotSerialServoReadMode(uint8_t _id, int16_t *_speed);
//    uint8_t LobotSerialServoReadError(uint8_t _id);/*! ret can be a value of @ref ErrorCode */
//    bool IsLobotSerialServoLoading(uint8_t _id);
//
//private:
//    static uint8_t LobotRxBuf[SERVO_RX_BUFF_LEN];
//private:
///** ------------------------------------- BASIC FUNCTIONS ---------------------------------------- **/
//    uint8_t LobotCheckSum(uint8_t buf[]) {
//        uint8_t i;
//        uint16_t temp = 0;
//        for (i = 2; i < buf[3] + 2; i++) {
//            temp += buf[i];
//        }
//        temp = ~temp;
//        i = (uint8_t) temp;
//        return i;
//    }
//
//    bool LobotReadCheck(uint8_t rx_buf[], uint8_t _id, uint8_t _data_len, uint8_t _instruction) {
//        if(unsigned_16(rx_buf) == 0x5555 &&
//           rx_buf[2] == _id && rx_buf[3] == _data_len &&
//           rx_buf[4] ==_instruction &&
//           LobotCheckSum(rx_buf) != rx_buf[rx_buf[3]+2])
//            return true;
//        else
//            return false;
//    }
//};
//
//
///*---------------------------- C Scope ---------------------------*/
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif //__DRV_HIWONDER_MOTOR_H
