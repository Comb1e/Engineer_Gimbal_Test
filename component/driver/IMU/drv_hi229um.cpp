//
// Created by Henson on 2023-11-18.
//

#include "drv_hi229um.h"

const char mode_cmd_91[] = "AT+SETPTL=91\r\n";
const char mode_cmd_90[] = "AT+SETPTL=90,A0,B0,C0,D0,F0\r\n";

const char mode_cmd_6_axis[] = "AT+MODE=0\r\n";
const char mode_cmd_9_axis[] = "AT+MODE=1\r\n";

const char dir_cmd_00[] = "AT+URFR=1,0,0,0,1,0,0,0,1\r\n";//陀螺仪

const char dir_cmd_x_90[] = "AT+URFR=1,0,0,0,0,1,0,-1,0\r\n";//x 90
const char dir_cmd_x_neg_90[] = "AT+URFR=1,0,0,0,0,-1,0,1,0\r\n";//x -90
const char dir_cmd_x_180[] = "AT+URFR=1,0,0,0,-1,0,0,0,-1\r\n";//x 180

const char dir_cmd_y_90[] = "AT+URFR=0,0,-1,0,1,0,1,0,0\r\n";//y 90
const char dir_cmd_y_neg_90[] = "AT+URFR=0,0,1,0,1,0,-1,0,0\r\n";//y -90
const char dir_cmd_y_180[] = "AT+URFR=-1,0,0,0,1,0,0,0,-1\r\n";//y 180

const char dir_cmd_z_90[] = "AT+URFR=0,1,0,-1,0,0,0,0,1\r\n";//z 90
const char dir_cmd_z_neg_90[] = "AT+URFR=0,-1,0,1,0,0,0,0,1\r\n";//z -90
const char dir_cmd_z_180[] = "AT+URFR=-1,0,0,0,-1,0,0,0,1\r\n";//z 180

const char dir_cmd_921600[] = "AT+BAUD=921600\r\n";
const char dir_cmd_460800[] = "AT+BAUD=460800\r\n";
const char dir_cmd_115200[] = "AT+BAUD=115200\r\n";

const char dir_cmd_rst[] = "AT+RST\r\n";
const char dir_cmd_400[] = "AT+ODR=400\r\n";
const char dir_cmd_x_180_y_90[] = "AT+URFR=0,0,1,0,-1,0,1,0,0\r\n";//x 180 y 90

// ------------------------------------ All Mode ------------------------------------ //

IMU_Hi229um::IMU_Hi229um() : IMU(){

}

void IMU_Hi229um::IMU_Hi229um_init(bool _is_using_91_mode,UART_HandleTypeDef *_huart,osSemaphoreId_t sem_rx){
    this->huart = _huart;
    this->is_using_91_mode = _is_using_91_mode;
    rx_semaphore = sem_rx;
}

void IMU_Hi229um::instruction_set(const char *instruction) {
    usart_send_buf(huart, (uint8_t *)instruction, strlen(instruction));
}

void IMU_Hi229um::update_single_gyro(IMU::imu_e imuE, float deg_ceiling) {
    gyro_t* gyro;
    switch (imuE){
        case x:
            gyro = &imu_data.gx;
            break;
        case y:
            gyro = &imu_data.gy;
            break;
        case z:
            gyro = &imu_data.gz;
            break;
        default:
            return ;
    }
    if (gyro->deg - gyro->last_deg > deg_ceiling) {
        gyro->round_cnt--;
        gyro->delta_deg = gyro->deg - gyro->last_deg - 2.0f * deg_ceiling;
    } else if (gyro->deg - gyro->last_deg < -deg_ceiling) {
        gyro->round_cnt++;
        gyro->delta_deg = gyro->deg - gyro->last_deg + 2.0f * deg_ceiling;
    } else {
        gyro->delta_deg = gyro->deg - gyro->last_deg;
    }
    gyro->total_rounds =
            (float) gyro->round_cnt * (deg_ceiling / 180.0f) + (1.0f / 360.0f) * (gyro->deg - gyro->offset_deg);
}


void IMU_Hi229um::update_gyro(){
    if(is_using_91_mode) {
        imu_data.gx.ang_vel = raw.data_91.ang_v.x;
        imu_data.gy.ang_vel = raw.data_91.ang_v.y;
        imu_data.gz.ang_vel = raw.data_91.ang_v.z;

        imu_data.gx.last_deg = imu_data.gx.deg;
        imu_data.gx.deg = raw.data_91.euler_ang.roll;

        imu_data.gy.last_deg = imu_data.gy.deg;
        imu_data.gy.deg = raw.data_91.euler_ang.pitch;

        imu_data.gz.last_deg = imu_data.gz.deg;
        imu_data.gz.deg = raw.data_91.euler_ang.yaw;
    }else{
        imu_data.gx.ang_vel = (float)raw.data_90.ang_v.x * 0.1f;
        imu_data.gy.ang_vel = (float)raw.data_90.ang_v.y * 0.1f;
        imu_data.gz.ang_vel = (float)raw.data_90.ang_v.z * 0.1f;

        imu_data.gx.last_deg = imu_data.gx.deg;
        imu_data.gx.deg = raw.data_90.euler_ang.roll * 0.01f;

        imu_data.gy.last_deg = imu_data.gy.deg;
        imu_data.gy.deg = raw.data_90.euler_ang.pitch * 0.01f;

        imu_data.gz.last_deg = imu_data.gz.deg;
        imu_data.gz.deg = raw.data_90.euler_ang.yaw * 0.1f;
    }
    imu_data.gx.integral_deg = (float)(imu_data.gx.ang_vel * delta_t_s);
    imu_data.gy.integral_deg = (float)(imu_data.gy.ang_vel * delta_t_s);
    imu_data.gz.integral_deg = (float)(imu_data.gz.ang_vel * delta_t_s);

    //roll
    update_single_gyro(x,180.0f);
    //pitch
    update_single_gyro(y,90.0f);
    //yaw
    update_single_gyro(z,180.0f);
}


void IMU_Hi229um::update_acc(){
    if(is_using_91_mode){
        imu_data.ax.acc = raw.data_91.acc.x * g ;
        imu_data.ay.acc = raw.data_91.acc.y * g ;
        imu_data.az.acc = raw.data_91.acc.z * g ;
    }
    else{
        imu_data.ax.acc = (float)raw.data_90.acc.x * 0.001f * g ;
        imu_data.ay.acc = (float)raw.data_90.acc.y * 0.001f * g ;
        imu_data.az.acc = (float)raw.data_90.acc.z * 0.001f * g ;
    }

    imu_data.ax.vel += (float)((imu_data.ax.acc - imu_data.ax.acc_offset) * delta_t_s);
    imu_data.ay.vel += (float)((imu_data.ay.acc - imu_data.ax.acc_offset) * delta_t_s);
    imu_data.az.vel += (float)((imu_data.az.acc - imu_data.ax.acc_offset) * delta_t_s);

    imu_data.ax.pos += (float)(imu_data.ax.vel * delta_t_s);
    imu_data.ay.pos += (float)(imu_data.ay.vel * delta_t_s);
    imu_data.az.pos += (float)(imu_data.az.vel * delta_t_s);
}

void IMU_Hi229um::update_msg(){
    __NOP();
}

void IMU_Hi229um::init_gyro_offset(){
    if(is_using_91_mode) {
        imu_data.gx.last_deg = imu_data.gx.deg;
        imu_data.gx.deg = raw.data_91.euler_ang.roll;
        imu_data.gx.offset_deg = imu_data.gx.deg;
        imu_data.gx.round_cnt = 0;

        imu_data.gy.last_deg = imu_data.gy.deg;
        imu_data.gy.deg = raw.data_91.euler_ang.pitch;
        imu_data.gy.offset_deg = imu_data.gy.deg;
        imu_data.gy.round_cnt = 0;

        imu_data.gz.last_deg = imu_data.gz.deg;
        imu_data.gz.deg = raw.data_91.euler_ang.yaw;
        imu_data.gz.offset_deg = imu_data.gz.deg;
        imu_data.gz.round_cnt = 0;
    }
    else{
        imu_data.gx.last_deg = imu_data.gx.deg;
        imu_data.gx.deg = (float)raw.data_90.euler_ang.roll * 0.01f;
        imu_data.gx.offset_deg = imu_data.gx.deg;
        imu_data.gx.round_cnt = 0;

        imu_data.gy.last_deg = imu_data.gy.deg;
        imu_data.gy.deg = (float)raw.data_90.euler_ang.pitch * 0.01f;
        imu_data.gy.offset_deg = imu_data.gy.deg;
        imu_data.gy.round_cnt = 0;

        imu_data.gz.last_deg = imu_data.gz.deg;
        imu_data.gz.deg = (float)raw.data_90.euler_ang.yaw * 0.1f;
        imu_data.gz.offset_deg = imu_data.gz.deg;
        imu_data.gz.round_cnt = 0;
    }
}

void IMU_Hi229um::init_acc_offset(){
    if(is_using_91_mode) {
        imu_data.ax.acc_offset = raw.data_91.acc.x * g;
        imu_data.ay.acc_offset = raw.data_91.acc.y * g;
        imu_data.az.acc_offset = raw.data_91.acc.z * g;
    }else{
        imu_data.ax.acc_offset = (float) raw.data_90.acc.x * 0.001f * g;
        imu_data.ay.acc_offset = (float) raw.data_90.acc.y * 0.001f * g;
        imu_data.az.acc_offset = (float) raw.data_90.acc.z * 0.001f * g;
    }
}

void IMU_Hi229um::update_imu() {
    update_frequency_Hz();
    taskENTER_CRITICAL();
    rcv_cnt++;
    if(last_offset==true && is_zero_offset==false){
        rcv_cnt = 0;
    }
    if (rcv_cnt>50) {
        is_zero_offset = true;
    }else{
        is_zero_offset = false;
    }
    last_offset = is_zero_offset;
    taskEXIT_CRITICAL();
    if (is_zero_offset==false) {
        init_gyro_offset();
        init_acc_offset();
        return;
    }
    update_gyro();
    update_acc();
    update_msg();
}


bool IMU_Hi229um::is_crc_passing() {
    uint16_t crc = 0;
    /* checksum */
    crc16_update(&crc, raw.buf, 4);
    crc16_update(&crc, raw.buf + 6, raw.frame_head.len);//
    if (crc != U2(raw.buf + 4)) {
        return false;
    }
    return true;
}

bool IMU_Hi229um::is_data_legal() {
    if (raw.frame_head.head == 0xa55a && this->is_crc_passing())
        return true;
    else
        return false;
}

void IMU_Hi229um::start_receive_dma() {
    HAL_UARTEx_ReceiveToIdle_DMA(huart,raw.buf,DRV_HI229UM_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
//    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC);
}

void IMU_Hi229um::reset_imu(){
    __NOP();
}



