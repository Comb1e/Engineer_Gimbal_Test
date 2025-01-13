//
// Created by Henson on 2024-03-07.
//

#include "drv_comm.h"

#include "usbd_cdc_if.h"

Comm_CAN comm_can;
Comm_USB comm_usb;
Communication communication;

usb_rx_raw_t rx_raw_data={};

Comm_CAN::Comm_CAN()
{
    memset(&this->tx_data, 0, sizeof(this->tx_data));
}

void Comm_CAN::Init(CAN_HandleTypeDef *hcan)
{
    this->can_device.can_init(hcan);
    this->can_device.can_rx_tx_add(GIMBAL_TO_CHASSIS_RX_CAN2_STDID,CommCanBinarySemHandle,
                                   [this](CAN *_can_dev,uint8_t *_rx_data) {this->Comm_Can_RX_Callback(_can_dev,_rx_data);},GIMBAL_TO_CHASSIS_TX_CAN2_STDID,(uint8_t *)&this->tx_data,0x08);
}

void Comm_CAN::Comm_Can_RX_Callback(CAN *can_device,uint8_t *data)
{
    Comm_CAN *comm_can = Container_Of(can_device,Comm_CAN,can_device);
    memcpy(&comm_can->rx_raw_data, data, sizeof(comm_can->rx_raw_data));
}

void Comm_USB::Init()
{

}

void Comm_USB::Start_Receive_Data()
{
    CDC_Receive_FS_Mine_Del((uint8_t *) &this->rx_raw, NULL);
}

void Comm_USB::Transmit_Data()
{
    CDC_Transmit_FS_Mine_Del((uint8_t *)&this->tx_data, USB_INFO_TX_BUF_NUM);
}

void Comm_USB::Update_RX_Data()
{
    auto rx_data = (usb_rx_raw_t *)usb_buf;
    this->rx_raw.arm_pitch_joint = rx_data->arm_pitch_joint;
    this->rx_raw.arm_roll_joint = rx_data->arm_roll_joint;
    this->rx_raw.arm_yaw_joint = rx_data->arm_yaw_joint;
    this->rx_raw.chassis_spin = rx_data->chassis_spin;
    this->rx_raw.chassis_x = rx_data->chassis_x;
    this->rx_raw.chassis_y = rx_data->chassis_y;
    this->rx_raw.extend_joint = rx_data->extend_joint;
    this->rx_raw.frame_head = rx_data->frame_head;
    this->rx_raw.frame_tail = rx_data->frame_tail;
    this->rx_raw.is_arm_pump_open = rx_data->is_arm_pump_open;
    this->rx_raw.is_chassis_vel_control = rx_data->is_chassis_vel_control;
    this->rx_raw.is_left_pump_open = rx_data->is_left_pump_open;
    this->rx_raw.is_right_pump_open = rx_data->is_right_pump_open;
    this->rx_raw.pitch_joint = rx_data->pitch_joint;
    this->rx_raw.roll_joint = rx_data->roll_joint;
    this->rx_raw.slide_joint = rx_data->slide_joint;
    this->rx_raw.uplift_joint = rx_data->uplift_joint;
}


Communication::Communication()
{

}

void Communication::PTR_Init(Comm_CAN *comm_can,Comm_USB *comm_usb,Arm *arm,rc_device_t *rc)
{
    this->can = comm_can;
    this->usb = comm_usb;
    this->arm = arm;
    this->rc = rc;
}

void Communication::Init()
{
    this->can->Init(&COMM_DATA_CAN);
    this->usb->Init();
}

void Communication::Update_CAN_TX_Data()
{
    this->can->tx_data.chassis_spin = this->usb->rx_raw.chassis_spin;
    this->can->tx_data.chassis_x = this->usb->rx_raw.chassis_x;
    this->can->tx_data.chassis_y = this->usb->rx_raw.chassis_y;
    this->can->tx_data.is_arm_pump_open = this->usb->rx_raw.is_arm_pump_open;
    this->can->tx_data.is_left_pump_open = this->usb->rx_raw.is_left_pump_open;
    this->can->tx_data.is_right_pump_open = this->usb->rx_raw.is_right_pump_open;
    this->can->tx_data.is_chassis_vel_control = this->usb->rx_raw.is_chassis_vel_control;
}


void Communication::Update_TX_Data()
{
    this->Update_CAN_TX_Data();

    this->usb->tx_data.frame_head = USB_FRAME_HEAD;
    memcpy(&this->usb->tx_data.remote_ctrl,&this->rc->raw,sizeof(this->usb->tx_data.remote_ctrl));
    this->usb->tx_data.arm_pitch_joint = this->arm->current_joint.arm_pitch_joint;
    this->usb->tx_data.arm_roll_joint = this->arm->current_joint.arm_roll_joint;
    this->usb->tx_data.arm_yaw_joint = this->arm->current_joint.arm_yaw_joint;
    this->usb->tx_data.extend_joint = this->arm->current_joint.extend_joint;
    this->usb->tx_data.pitch_joint = this->arm->current_joint.pitch_joint;
    this->usb->tx_data.roll_joint = this->arm->current_joint.roll_joint;
    this->usb->tx_data.slide_joint = this->arm->current_joint.slide_joint;
    this->usb->tx_data.uplift_joint = this->arm->current_joint.uplift_joint;
    this->usb->tx_data.is_arm_pump_holding_on = this->can->rx_raw_data.is_arm_pump_holding_on;
    this->usb->tx_data.is_left_pump_holding_on = this->can->rx_raw_data.is_left_pump_holding_on;
    this->usb->tx_data.is_right_pump_holding_on = this->can->rx_raw_data.is_right_pump_holding_on;
    this->usb->tx_data.is_rc_online = !this->rc->is_lost;
    this->usb->tx_data.chassis_gyro_totoal_rounds = this->can->rx_raw_data.chassis_gyro_totoal_rounds;
    this->usb->tx_data.frame_tail = USB_FRAME_TAIL;
}

void Communication::Update_USB_Lost_Flag()
{
    osStatus_t status_can = osSemaphoreAcquire(USBUpdateBinarySemHandle,30);
    if(status_can == osOK)
    {
        this->usb_lost_flag = false;
    }
    else
    {
        this->usb_lost_flag = true;
    }
}

void Communication::Update_CAN_Lost_Flag()
{
    osStatus_t status_can = osSemaphoreAcquire(this->can->can_device.rx.semaphore,30);
    if(status_can == osOK)
    {
        this->can_lost_flag = false;
    }
    else
    {
        this->can_lost_flag = true;
    }
}

void Communication::Update_Chassis_Control_RC()
{
    /*this->can->tx_data.chassis_spin = this->rc->rc_info.rocker.right_x;
    this->can->tx_data.chassis_x = this->rc->rc_info.rocker.left_y;
    this->can->tx_data.chassis_y = -this->rc->rc_info.rocker.left_x;*/
    this->can->tx_data.chassis_spin = 0;
    this->can->tx_data.chassis_x = 0;
    this->can->tx_data.chassis_y = 0;
    this->can->tx_data.is_arm_pump_open = 0;
    this->can->tx_data.is_left_pump_open = 0;
    this->can->tx_data.is_right_pump_open = 0;
    this->can->tx_data.is_chassis_vel_control = 0;
    /*
    this->can->tx_data.is_chassis_vel_control = 1;
*/
}


bool Communication::Check_USB_Lost_Flag()
{
    return this->usb_lost_flag || this->can_lost_flag;
}

void Communication::Send_MSG()
{
    this->can->can_device.send_can_msg();
    this->usb->Transmit_Data();
}

void Communication::Update_Arm_Control()
{
    this->arm->set_joint.arm_pitch_joint = this->usb->rx_raw.arm_pitch_joint;
    this->arm->set_joint.arm_roll_joint = this->usb->rx_raw.arm_roll_joint;
    this->arm->set_joint.arm_yaw_joint = this->usb->rx_raw.arm_yaw_joint;
    this->arm->set_joint.extend_joint = this->usb->rx_raw.extend_joint;
    this->arm->set_joint.pitch_joint = this->usb->rx_raw.pitch_joint;
    this->arm->set_joint.roll_joint = this->usb->rx_raw.roll_joint;
    this->arm->set_joint.slide_joint = this->usb->rx_raw.slide_joint;
    this->arm->set_joint.uplift_joint = this->usb->rx_raw.uplift_joint;
}
