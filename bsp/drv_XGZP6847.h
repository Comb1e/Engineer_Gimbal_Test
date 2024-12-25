//
// Created by Henson on 2024-05-23.
//

#ifndef __DRV_XGZP6847_H
#define __DRV_XGZP6847_H

/*---------------------------- C++ Scope ---------------------------*/
#include "freertos_inc.h"
#include "user_lib.h"

#define DELAY_TIME 800

class IIC{
public:
    typedef enum{
        low = 0,
        high = 1,
    }gpio_level_e;
    GPIO_TypeDef *GPIOx=nullptr;
    uint16_t SCL=0;
    uint16_t SDA=0;
public:
    IIC(){};

    void IIC_init(GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN);

    void IIC_Start();
    void IIC_Stop();
    void set_SDA(gpio_level_e level);
    void set_SCL(gpio_level_e level);
    bool read_SDA();

    bool is_receive_ack();

    void write_byte(uint8_t txd);
    uint8_t read_byte(uint8_t ack);

    void write_byte_to_addr(uint8_t addr, uint8_t byte);
    uint8_t read_byte_from_addr(uint8_t addr);

    void send_ack();
    void not_send_ack();

    void send_1_bit();
    void send_0_bit();

    uint8_t ReadI2CByte();

    void set_sda_output_mode();
    void set_sda_input_mode();

    //----delay time_us----
    void DELAY(uint32_t t) {
        while (t != 0)
            t--;
    }
};


class XGZP6847{
public:
    IIC iic;
    float pressure = 0;//pa
    float temperature = 0;//℃
    //用于保存校准后的压力值和温度值
    uint8_t pressure_H=0, pressure_M=0, pressure_L=0, temperature_H=0, temperature_L=0;//临时变量，用于保存从传感器中读出的与压力和温度相关的寄存器的数值
    int32_t pressure_adc=0, temperature_adc=0;//临时变量，用于保存传感器 ADC 转换后的压力值和温度值
    uint32_t update_cnt = 0;
public:
    XGZP6847():iic(IIC()){};
    void XGZP6847_init(GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN);
    void update_data();//10Hz

    double get_pressure();
    double get_temperature();
};


/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void XGZP6847Task(void *argument);


#ifdef __cplusplus
}
#endif

#endif //__DRV_XGZP6847_H
