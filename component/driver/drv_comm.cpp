//
// Created by Henson on 2024-03-07.
//

#include "drv_comm.h"

#include "usbd_cdc_if.h"

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

void Comm_USB::Receive_Data()
{
    if (USBD_OK == CDC_Receive_FS_Mine_Del((uint8_t *) &this->rx_raw, NULL))
    {
        osSemaphoreRelease(USBUpdateBinarySemHandle);
        __NOP();
    }
    else
    {
        __NOP();
    }
}

void Comm_USB::Transmit_Data()
{
    CDC_Transmit_FS_Mine_Del((uint8_t *)&this->tx_data, USB_INFO_TX_BUF_NUM);
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

void Communication::Update_Lost_Flag()
{
    osStatus_t status_can = osSemaphoreAcquire(this->can->can_device.rx.semaphore,15);
    osStatus_t status_usb = osSemaphoreAcquire(USBUpdateBinarySemHandle,15);
    if(status_can == osOK && status_usb == osOK)
    {
        this->lost_flag = false;
    }
    else
    {
        this->lost_flag = true;
    }
}

bool Communication::Check_Lost()
{
    return this->lost_flag;
}

void Communication::Send_MSG()
{
    this->can->can_device.send_can_msg();
    this->usb->Transmit_Data();
}
