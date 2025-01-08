//
// Created by Henson on 2024-03-07.
//

#include "drv_comm.h"

Comm_CAN comm_can;
Comm_USB comm_usb;
Communication communication;

Comm_CAN::Comm_CAN()
{

}

void Comm_CAN::Init(CAN_HandleTypeDef *hcan)
{
    this->can_device.can_init(hcan);
    this->can_device.can_rx_tx_add(GIMBAL_TO_CHASSIS_RX_CAN2_STDID,CommCanBinarySemHandle,Comm_Can_RX_Callback,GIMBAL_TO_CHASSIS_TX_CAN2_STDID,(uint8_t *)&this->tx_data,0x08);
}

void Comm_Can_RX_Callback(CAN *can_device,uint8_t *data)
{
    Comm_CAN *comm_can = Container_Of(can_device,Comm_CAN,can_device);
    memcpy(&comm_can->rx_raw_data, data, sizeof(comm_can->rx_raw_data));
}

void Comm_USB::Init()
{

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

void Communication::Update_Data()
{
    this->can->tx_data.chassis_spin = this->usb->rx_raw.chassis_spin;
    this->can->tx_data.chassis_x = this->usb->rx_raw.chassis_x;
    this->can->tx_data.chassis_y = this->usb->rx_raw.chassis_y;
    this->can->tx_data.is_arm_pump_open = this->usb->rx_raw.is_arm_pump_open;
    this->can->tx_data.is_left_pump_open = this->usb->rx_raw.is_left_pump_open;
    this->can->tx_data.is_right_pump_open = this->usb->rx_raw.is_right_pump_open;
    this->can->tx_data.is_chassis_vel_control = this->usb->rx_raw.is_chassis_vel_control;

    this->usb->tx_data.frame_head = USB_FRAME_HEAD;
    memcpy(&this->usb->tx_data.remote_ctrl,&this->rc->raw,sizeof(this->usb->tx_data.remote_ctrl));
    //this->usb->tx_data.arm_pitch_joint = this->arm->actuator.
    this->usb->tx_data.frame_tail = USB_FRAME_TAIL;
}
