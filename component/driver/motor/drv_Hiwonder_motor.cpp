////
//// Created by Henson on 2024-03-07.
////
//
//#include "drv_Hiwonder_motor.h"
//#include "user_lib.h"
//
////要解决收发不能同时READ中可能要改
//#define LobotSerialWrite    usart_send_buf_dma
//#define LobotSerialRead     usart_idle_receive_buf
//
//Hiwonder_Motor::Hiwonder_Motor(UART_HandleTypeDef *huart) {
//    this->huart = huart;
//    is_enable = true;
//}
//
//int8_t Hiwonder_Motor::get_temperature(uint8_t _id) {
//    return (int8_t)LobotSerialServoReadTemp(_id);
//}
//
//void Hiwonder_Motor::set_motor_free(uint8_t _id){
//    LobotSerialServoSetUnload(_id);
//}
//
//bool Hiwonder_Motor::LobotSerialDeviceAdd(uint8_t _dev_index, uint8_t _id) {
//    if(_dev_index < LOBOT_SERVO_NUM) {
//        servo_dev[_dev_index].id = _id;
//        servo_dev[_dev_index].is_servo = true;
//        servo_dev[_dev_index].is_load = true;
//        servo_dev[_dev_index].is_engine = false;
//        id2dev[servo_dev[_dev_index].id] = &servo_dev[_dev_index];
//        return true;
//    }
//    return false;
//}
//
//
////_position 0~1
//void Hiwonder_Motor::LobotSerialServoSetPosf(uint8_t _id, float _position_f, uint16_t _time_ms){
//    int16_t pos = (int16_t)(_position_f*LOBOT_SERVO_POS_MAX);
//    LobotSerialServoMoveSet(_id, pos, _time_ms);
//}
//
//
////_ang 0~240
//void Hiwonder_Motor::LobotSerialServoSetAng(uint8_t _id, float _ang_f, uint16_t _time_ms){
//    int16_t pos = (int16_t)(_ang_f / LOBOT_SERVO_ANG_MAX * LOBOT_SERVO_POS_MAX);
//    LobotSerialServoMoveSet(_id, pos, _time_ms);
//}
//
////speed -1~1 不支持掉电保存
//void Hiwonder_Motor::LobotSerialServoSetEngineSpeedf(uint8_t _id, float _speed_f) {
//    int16_t _speed = (int16_t)(_speed_f * LOBOT_ENGINE_SPEED_MAX);
//    LobotSerialServoSetEngineMode(_id,_speed);
//}
//
////_ang 0~240 支持掉电保存
//void Hiwonder_Motor::LobotSerialServoAngLimit(uint8_t _id, float _ang_min_f, float _ang_max_f){
//    uint16_t _pos_min,_pos_max;
//    if(_ang_min_f<=_ang_max_f) {
//        if(_ang_max_f>240.0f) _ang_max_f = 240.0f;
//        if(_ang_min_f<0.0f) _ang_min_f = 0.0f;
//        _pos_min = (uint16_t) (_ang_min_f / LOBOT_SERVO_ANG_MAX * LOBOT_SERVO_POS_MAX);
//        _pos_max = (uint16_t) (_ang_max_f / LOBOT_SERVO_ANG_MAX * LOBOT_SERVO_POS_MAX);
//        LobotSerialServoPosLimit(_id,_pos_min,_pos_max);
//    }
//}
//
//
////_ang -30~30 不支持掉电保存
//void Hiwonder_Motor::LobotSerialServoChangeAngOffset(uint8_t _id, float  _offset_ang_f){
//    int8_t _offset_pos = 0;
//    if(_offset_ang_f>30.0f) _offset_ang_f = 30.0f;
//    if(_offset_ang_f<-30.0f) _offset_ang_f = -30.0f;
//    _offset_pos = (int8_t)(_offset_ang_f / LOBOT_SERVO_ANG_MAX * LOBOT_SERVO_POS_MAX);
//    LobotSerialServoChangePosOffset(_id, _offset_pos);
//}
//
//
////_ang -30~30 掉电保存
//void Hiwonder_Motor::LobotSerialServoSetSaveAngOffset(uint8_t _id, float _offset_ang_f) {
//    LobotSerialServoChangeAngOffset(_id,_offset_ang_f);
//    LobotSerialServoSaveOffset(_id);
//}
//
////_pos -125~125 掉电保存
//void Hiwonder_Motor::LobotSerialServoSetSavePosOffset(uint8_t _id, int8_t _offset_pos) {
//    LobotSerialServoChangePosOffset(_id,_offset_pos);
//    LobotSerialServoSaveOffset(_id);
//}
//
////总线上只有一个舵机时使用其读ID
//uint8_t Hiwonder_Motor::LobotSerialReadID() {
//    return LobotSerialServoReadID(LOBOT_BROADCAST_ID);
//}
//
///** ------------------------------------- WRITE ---------------------------------------- **/
////1 0-1000 0-30000
//void Hiwonder_Motor::LobotSerialServoMoveSet( uint8_t _id, int16_t _position, uint16_t _time_ms) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[10];//static,这个用了static后如果不用osdelay就会只动一个爪子
//    if(_position<0) _position = 0;
//    if(_position>1000) _position = 1000;
//    _time_ms = _time_ms>30000?30000:_time_ms;
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 7;
//    _buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
//    _buf[5] = low_byte((uint16_t)_position);
//    _buf[6] = high_byte((uint16_t)_position);
//    _buf[7] = low_byte(_time_ms);
//    _buf[8] = high_byte(_time_ms);
//    _buf[9] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 10);
//    _servo_dev->servo_mode.current_pos = _position;
//    _servo_dev->servo_mode.current_ang = (float)_servo_dev->servo_mode.current_pos * LOBOT_SERVO_POS2ANG;
//}
//
////7 0-1000  0-30000ms
//void Hiwonder_Motor::LobotSerialServoPreMoveSet( uint8_t _id, int16_t _pre_pos, uint16_t _pre_time_ms) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[10];
//    if(_pre_pos<0) _pre_pos = 0;
//    if(_pre_pos>1000) _pre_pos = 1000;
//    _pre_time_ms = _pre_time_ms>30000?30000:_pre_time_ms;
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 7;
//    _buf[4] = LOBOT_SERVO_MOVE_TIME_WAIT_WRITE;
//    _buf[5] = low_byte((uint16_t)_pre_pos);
//    _buf[6] = high_byte((uint16_t)_pre_pos);
//    _buf[7] = low_byte(_pre_time_ms);
//    _buf[8] = high_byte(_pre_time_ms);
//    _buf[9] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 10);
//    _servo_dev->servo_mode.pre_pos = (uint16_t)_pre_pos;
//    _servo_dev->servo_mode.pre_ang = (float)_servo_dev->servo_mode.pre_pos * LOBOT_SERVO_POS2ANG;
//    _servo_dev->servo_mode.is_need_start = true;
//}
//
////11
//void Hiwonder_Motor::LobotSerialServoStart(uint8_t _id) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_MOVE_START;
//    _buf[5] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 6);
//    _servo_dev->servo_mode.current_pos = (int16_t)_servo_dev->servo_mode.pre_pos;
//    _servo_dev->servo_mode.current_ang = _servo_dev->servo_mode.pre_ang;
//    _servo_dev->servo_mode.is_need_start = false;
//}
//
//
////12
//void Hiwonder_Motor::LobotSerialServoStop(uint8_t _id) {
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_MOVE_STOP;
//    _buf[5] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 6);
//}
//
//
////13 0~253 默认为1 掉电保存
//void Hiwonder_Motor::LobotSerialServoSetID(uint8_t oldID, uint8_t newID) {
//    servo_device_t *_servo_dev = id2dev[oldID];
//    id2dev[oldID] = NULL;//原来的设为空
//    uint8_t _buf[7];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = oldID;
//    _buf[3] = 4;
//    _buf[4] = LOBOT_SERVO_ID_WRITE;
//    _buf[5] = newID;
//    _buf[6] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 7);
//    _servo_dev->id = newID;
//    id2dev[_servo_dev->id] = _servo_dev;
//}
//
//
////17 -125~125 不支持掉电保存
//void Hiwonder_Motor::LobotSerialServoChangePosOffset(uint8_t _id, int8_t _offset_pos) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[7];
//    if(_offset_pos>125) _offset_pos = 125;
//    if(_offset_pos<-125) _offset_pos = -125;
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 4;
//    _buf[4] = LOBOT_SERVO_ANGLE_OFFSET_ADJUST;
//    _buf[5] = (uint8_t)_offset_pos;
//    _buf[6] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 7);
//    _servo_dev->servo_mode.pos_offset = _offset_pos;
//    _servo_dev->servo_mode.ang_offset = (float)_servo_dev->servo_mode.pos_offset * LOBOT_SERVO_POS2ANG;
//}
//
////18
//void Hiwonder_Motor::LobotSerialServoSaveOffset(uint8_t _id) {
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_ANGLE_OFFSET_WRITE;
//    _buf[5] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 6);
//}
//
//
////20支持掉电保存  0-1000
//void Hiwonder_Motor::LobotSerialServoPosLimit(uint8_t _id, uint16_t _pos_min, uint16_t _pos_max) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[10];
//    _pos_max = _pos_max>1000?1000:_pos_max;
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 7;
//    _buf[4] = LOBOT_SERVO_ANGLE_LIMIT_WRITE;
//    _buf[5] = low_byte(_pos_min);
//    _buf[6] = high_byte(_pos_min);
//    _buf[7] = low_byte(_pos_max);
//    _buf[8] = high_byte(_pos_max);
//    _buf[9] = LobotCheckSum(_buf);
//    if(_pos_min<_pos_max){
//        LobotSerialWrite(huart, _buf, 10);
//        _servo_dev->servo_mode.pos_max = _pos_max;
//        _servo_dev->servo_mode.pos_min = _pos_min;
//    }
//
//}
//
////22支持掉电保存  4500-12000mV
//void Hiwonder_Motor::LobotSerialServoVoltageLimit( uint8_t _id, uint16_t _vol_min, uint16_t _vol_max) {
//    uint8_t _buf[10];
//    _vol_min = _vol_min<4500?4500:_vol_min;
//    _vol_max = _vol_max>12000?12000:_vol_max;
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 7;
//    _buf[4] = LOBOT_SERVO_VIN_LIMIT_WRITE;
//    _buf[5] = low_byte(_vol_min);
//    _buf[6] = high_byte(_vol_min);
//    _buf[7] = low_byte(_vol_max);
//    _buf[8] = high_byte(_vol_max);
//    _buf[9] = LobotCheckSum(_buf);
//    if(_vol_min<_vol_max){
//        LobotSerialWrite(huart, _buf, 10);
//    }
//
//}
//
////24支持掉电保存 50~100  默认85℃
//void Hiwonder_Motor::LobotSerialServoTempLimit(uint8_t _id, uint8_t _temp_max) {
//    uint8_t _buf[7];
//    _temp_max = _temp_max<50?50:(_temp_max>100?100:_temp_max);
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 4;
//    _buf[4] = LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE;
//    _buf[5] = _temp_max;
//    _buf[6] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 7);
//}
//
//
////29 -1000-1000 不支持掉电保存
//void Hiwonder_Motor::LobotSerialServoSetEngineMode(uint8_t _id, int16_t _speed) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[10];
//    if(_speed>1000) _speed = 1000;
//    if(_speed<-1000) _speed = -1000;
//    if(_servo_dev->engine_mode.is_reverse)  _speed = -_speed;
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 7;
//    _buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
//    _buf[5] = 1;
//    _buf[6] = 0;
//    _buf[7] = low_byte((uint16_t)_speed);
//    _buf[8] = high_byte((uint16_t)_speed);
//    _buf[9] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 10);
//    _servo_dev->is_engine = true;
//    _servo_dev->is_servo = false;
//    _servo_dev->engine_mode.speedf = (float)_speed/1000.0f;
//}
//
////29 不支持掉电保存
//void Hiwonder_Motor::LobotSerialServoSetServoMode(uint8_t _id) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[10];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 7;
//    _buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
//    _buf[5] = 0;
//    _buf[6] = _buf[7] = _buf[8] = 0;
//    _buf[9] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 10);
//    _servo_dev->is_servo = true;
//    _servo_dev->is_engine = false;
//}
//
//
////31
//void Hiwonder_Motor::LobotSerialServoSetUnload(uint8_t _id) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[7];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 4;
//    _buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
//    _buf[5] = 0;
//    _buf[6] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 7);
//    _servo_dev->is_load = false;
//}
//
//
////31
//void Hiwonder_Motor::LobotSerialServoSetLoad(uint8_t _id) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t _buf[7];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 4;
//    _buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
//    _buf[5] = 1;
//    _buf[6] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 7);
//    _servo_dev->is_load = true;
//}
//
//
////35
//void Hiwonder_Motor::LobotSerialServoErrorCfg(uint8_t _id, uint8_t _error_type) {
//    uint8_t _buf[7];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 4;
//    _buf[4] = LOBOT_SERVO_LED_ERROR_WRITE;
//    _buf[5] = _error_type;
//    _buf[6] = LobotCheckSum(_buf);
//    LobotSerialWrite(huart, _buf, 7);
//}
//
//
//
///** ----------------------------------- READ ---------------------------------------- **/
////这里也许要用到判断
////if (DR16_BUFF_LEN - __HAL_DMA_GET_COUNTER(_rc_dev->_huart->hdmarx) == 18)
//
////14 _id应为254 只能有一个舵机
//uint8_t Hiwonder_Motor::LobotSerialServoReadID(uint8_t _id) {
////    static osStatus_t stat = osError;
//    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_ID_READ;
//    _buf[5] = LobotCheckSum(_buf);
////    while(stat!=osOK && error_cnt < 8){
////        LobotSerialWrite(&SERVO_UART, _buf, 6);
////        LobotSerialRead(&SERVO_UART,rec_buf,rec_data_len+3);
////        stat = osSemaphoreAcquire(serialServoRecBinarySemHandle, 30);
////        if(stat==osOK && !LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_ID_READ)){
////            stat = osError;
////        }
////        error_cnt++;
////    }
//    while(!LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_ID_READ) && error_cnt < 30 ){
//        LobotSerialWrite(huart, _buf, 6);
//        LobotSerialRead(huart, rec_buf, SERVO_RX_BUFF_LEN);//rec_data_len+3
//        //这个地方可以大一点,或者是取最大即可,给rec_buf可以直接给最大,不用特别标准
//        error_cnt++;
//    }
//    if(error_cnt<30)
//        return rec_buf[5];//ID
//    else
//        return UINT8_ERROR_RETURN;
//}
//
//
////19 -125~125
//int8_t Hiwonder_Motor::LobotSerialServoReadPosOffset(uint8_t _id) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_ANGLE_OFFSET_READ;
//    _buf[5] = LobotCheckSum(_buf);
//
//    while(!LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_ANGLE_OFFSET_READ) && error_cnt < 30){
//        LobotSerialWrite(huart, _buf, 6);
//        LobotSerialRead(huart, rec_buf, SERVO_RX_BUFF_LEN);//rec_data_len+3
//        error_cnt++;
//    }
//    if(error_cnt<30){
//        _servo_dev->servo_mode.pos_offset = (int8_t)rec_buf[5];//Offset
//        return (int8_t)rec_buf[5];//Offset
//    }
//    else
//        return INT8_ERROR_RETURN;
//}
//
//
//
////26
//uint8_t Hiwonder_Motor::LobotSerialServoReadTemp(uint8_t _id) {
//    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_TEMP_READ;
//    _buf[5] = LobotCheckSum(_buf);
//
//    while(!LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_TEMP_READ) && error_cnt < 30){
//        LobotSerialWrite(huart, _buf, 6);
//        LobotSerialRead(huart, rec_buf, SERVO_RX_BUFF_LEN);//rec_data_len+3
//        error_cnt++;
//    }
//    if(error_cnt<30)
//        return rec_buf[5];//Temp
//    else
//        return UINT8_ERROR_RETURN;
//}
//
//
////27
//int16_t Hiwonder_Motor::LobotSerialServoReadVoltage(uint8_t _id) {
//    uint8_t rec_data_len = 5, rec_buf[7] = {0}, error_cnt = 0;
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_VIN_READ;
//    _buf[5] = LobotCheckSum(_buf);
//
//    while(!LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_VIN_READ) && error_cnt < 30){
//        LobotSerialWrite(huart, _buf, 6);
//        LobotSerialRead(huart, rec_buf, SERVO_RX_BUFF_LEN);//rec_data_len+3
//        error_cnt++;
//    }
//    if(error_cnt < 30)
//        return (int16_t)merge_bytes(rec_buf[6],rec_buf[5]);
//    else
//        return INT16_ERROR_RETURN;
//}
//
//
////28读出角度可能为负值
//int16_t Hiwonder_Motor::LobotSerialServoReadPosition(uint8_t _id) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t rec_data_len = 5, rec_buf[7] = {0}, error_cnt = 0;
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_POS_READ;
//    _buf[5] = LobotCheckSum(_buf);
//
//    while(!LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_POS_READ) && error_cnt < 30){
//        LobotSerialWrite(huart, _buf, 6);
//        LobotSerialRead(huart, rec_buf, SERVO_RX_BUFF_LEN);//rec_data_len+3
//        error_cnt++;
//    }
//    if(error_cnt < 30){
//        _servo_dev->servo_mode.current_pos = (int16_t)merge_bytes(rec_buf[6],rec_buf[5]);
//        return _servo_dev->servo_mode.current_pos;
//    }
//    else
//        return INT16_ERROR_RETURN;
//}
//
//
////30 speed可写为NULL
//uint8_t Hiwonder_Motor::LobotSerialServoReadMode(uint8_t _id, int16_t *_speed) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t rec_data_len = 7, rec_buf[7] = {0}, error_cnt = 0;
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_READ;
//    _buf[5] = LobotCheckSum(_buf);
//
//    while(!LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_OR_MOTOR_MODE_READ) && error_cnt < 30){
//        LobotSerialWrite(huart, _buf, 6);
//        LobotSerialRead(huart, rec_buf, SERVO_RX_BUFF_LEN);//rec_data_len+3
//        error_cnt++;
//    }
//
//    if(rec_buf[5] == LOBOT_SERVO_MODE_ENGINE && error_cnt < 30){
//        *_speed = (int16_t)merge_bytes(rec_buf[8],rec_buf[7]);
//        _servo_dev->is_servo = false;
//        _servo_dev->is_engine = true;
//        return 1;
//    }
//    else if(rec_buf[5] == LOBOT_SERVO_MODE_SERVO && error_cnt < 30){
//        _servo_dev->is_servo = true;
//        _servo_dev->is_engine = false;
//        return 0;
//    }
//    return UINT8_ERROR_RETURN;
//}
//
//
////32 输出可能是错误
//bool Hiwonder_Motor::IsLobotSerialServoLoading(uint8_t _id) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_READ;
//    _buf[5] = LobotCheckSum(_buf);
//
//    while(!LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_LOAD_OR_UNLOAD_READ) && error_cnt < 30){
//        LobotSerialWrite(huart, _buf, 6);
//        LobotSerialRead(huart, rec_buf, SERVO_RX_BUFF_LEN);//rec_data_len+3
//        error_cnt++;
//    }
//
//    if(rec_buf[5]==1 && error_cnt<30){
//        _servo_dev->is_load = true;
//        return true;
//    }
//    else
//        return false;
//}
//
//
////36
//uint8_t Hiwonder_Motor::LobotSerialServoReadError(uint8_t _id) {
//    servo_device_t *_servo_dev = id2dev[_id];
//    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
//    uint8_t _buf[6];
//    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
//    _buf[2] = _id;
//    _buf[3] = 3;
//    _buf[4] = LOBOT_SERVO_LED_ERROR_READ;
//    _buf[5] = LobotCheckSum(_buf);
//
//    while(!LobotReadCheck(rec_buf,_id,rec_data_len,LOBOT_SERVO_LED_ERROR_READ) && error_cnt < 30) {
//        LobotSerialWrite(huart, _buf, 6);
//        LobotSerialRead(huart, rec_buf, SERVO_RX_BUFF_LEN);//rec_data_len+3
//        error_cnt++;
//    }
//    if(error_cnt<30){
//        _servo_dev->error_code = rec_buf[5];
//        return rec_buf[5];
//    }
//
//    else
//        return UINT8_ERROR_RETURN;
//}
