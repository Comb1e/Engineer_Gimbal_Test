//
// Created by Henson on 2024-05-26.
//

#include "drv_XGZP6847.h"
#include "GlobalCfg.h"
XGZP6847 g_XGZP6847;


//超量程了会有错误数据
//void XGZP6847Task(void *argument) {
//    g_XGZP6847.XGZP6847_init(IIC_GPIO,IIC_SCL_PIN,IIC_SDA_PIN);
//    osDelay(1000);
//    for(;;) {
//        g_XGZP6847.update_data();
//        osDelay(100);
//    }
//}

void XGZP6847::XGZP6847_init(GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN){
    iic.IIC_init(GPIO,SCL_PIN,SDA_PIN);
}

double XGZP6847::get_pressure(){
    return pressure;
}

double XGZP6847::get_temperature(){
    return temperature;
}

// 感觉超量程了
//in loop
void XGZP6847::update_data(){
    float temp = 0;
    update_cnt++;
    //30
    iic.write_byte_to_addr(0x30, 0x0A);//001即可
    //indicate a combined conversion (once temperature conversion immediately followed by once sensor signal conversion)
    //0x30 里写入测量命令，000：单次温度测量；001：单次压力测量；010：组合：单次压力和温度测量；011：休眠方式（以一定的时间间隔执行组合模式测量）
    while((iic.read_byte_from_addr(0x30) & 0x08)>0);//30
    //Judge whether Data collection is over 判断数据采集是否结束
    pressure_H = iic.read_byte_from_addr(0x06);
    pressure_M = iic.read_byte_from_addr(0x07);
    pressure_L = iic.read_byte_from_addr(0x08);
    // Read ADC output Data of Pressure 读取保存压力值的 3 个寄存器的值
    pressure_adc = pressure_H * 65536 + pressure_M * 256 + pressure_L;
    //Compute the value of pressure converted by ADC 计算传感器 ADC 转换后的压力值
    if (pressure_adc > 8388608) //超过 8388608 为负压值，需在显示终端做正负号处理  4194304  这里 和说明书不太一样
    {
//        temp = (float)((pressure_adc & 0xefffff)-16777216)/64.0f;
        temp = ((float)pressure_adc - 16777216.0f) / 64.0f; //单位为 Pa
        if(temp>=-100000.0f){
            pressure = temp;
        }
    }else{
        temp = (float)pressure_adc / 64.0f;//单位为 Pa
        if(temp<=100000.0f){
            pressure = temp;
        }
    }
    VAL_LIMIT(pressure,-100000.0f,100000.0f);
    //The conversion formula of calibrated pressure，its unit is Pa 计算最终校准后的压力值

    temperature_H = iic.read_byte_from_addr(0x09);
    temperature_L = iic.read_byte_from_addr(0x0A);
    //Read ADC output data of temperature 读取保存温度值的 2 个寄存器的值
    temperature_adc = temperature_H * 256 + temperature_L;
    //Compute the value of temperature converted by ADC 计算传感器 ADC 转换后的压力温度值
    if(temperature_adc>32768) {
        temperature = ((float)temperature_adc - 65536.0f) / 256.0f; //单位为摄氏度
    }else{
        temperature = (float)temperature_adc / 256.0f; //单位为摄氏度
    }
    //The conversion formula of calibrated temperature, its unit is Centigrade 计算最终校准后的温度值
}


void IIC::IIC_init(GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN){
    GPIOx = GPIO;
    SCL = SCL_PIN;
    SDA = SDA_PIN;
}


void IIC::set_sda_output_mode(){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


void IIC::set_sda_input_mode(){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void IIC::set_SDA(gpio_level_e level){
    if(level==high){
        HAL_GPIO_WritePin(GPIOx,SDA, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOx, SDA, GPIO_PIN_RESET);

    }else{//low
        HAL_GPIO_WritePin(GPIOx, SDA, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(GPIOx,SDA, GPIO_PIN_SET);
    }
}

void IIC::set_SCL(gpio_level_e level){
    if(level==high){
        HAL_GPIO_WritePin(GPIOx, SCL, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOx, SCL, GPIO_PIN_RESET);
    }else{//low
        HAL_GPIO_WritePin(GPIOx, SCL, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(GPIOx, SCL, GPIO_PIN_SET);
    }
}

void IIC::IIC_Start() {
    set_sda_output_mode();
    set_SDA(high);
    DELAY(DELAY_TIME);
    set_SCL(high);
    DELAY(DELAY_TIME);
    set_SDA(low);//START:when CLK is high,DATA change form high to low
    DELAY(DELAY_TIME);
    set_SCL(low);//钳住I2C总线，准备发送或接收数据
    DELAY(DELAY_TIME);
}


void IIC::IIC_Stop() {
    set_sda_output_mode();
    set_SDA(low);//SDA OUTPUT LOW
    DELAY(DELAY_TIME);
    set_SCL(high);
    DELAY(DELAY_TIME);
    set_SDA(high);
    DELAY(DELAY_TIME);
    set_SCL(low);//SCL OUTPUT LOW
    DELAY(DELAY_TIME);
}

void IIC::send_1_bit(){
    set_sda_output_mode();
    set_SDA(high);
    DELAY(DELAY_TIME);
    set_SCL(high);
    DELAY(DELAY_TIME);
    set_SCL(low);
    DELAY(DELAY_TIME);
}

void IIC::send_0_bit(){
    set_sda_output_mode();
    set_SDA(low);
    DELAY(DELAY_TIME);
    set_SCL(high);
    DELAY(DELAY_TIME);
    set_SCL(low);
    DELAY(DELAY_TIME);
}

bool IIC::read_SDA() {
    if(HAL_GPIO_ReadPin(GPIOx, SDA)==GPIO_PIN_SET){
        return true;
    }else{
        return false;
    }
}

bool IIC::is_receive_ack(){
    uint16_t ucErrTime = 0;
    set_sda_input_mode();//SDA设置为输入
    set_SDA(high);
    DELAY(DELAY_TIME);
    set_SCL(high);
    DELAY(DELAY_TIME);
    while (read_SDA()) {
        ucErrTime++;
        if (ucErrTime > 250) {
            IIC_Stop();
            return false;
        }
    }
    set_SCL(low);//时钟输出0
    return true;
//    bool F0 = false;
//    set_SDA(high);
//    DELAY(DELAY_TIME);
//    set_SCL(high);
//    DELAY(DELAY_TIME / 2);
//    F0 = read_SDA();
//    DELAY(DELAY_TIME / 2);
//    set_SCL(low);
//    DELAY(DELAY_TIME);
//    if (F0 == true)
//        return false;
//    return true;
}

void IIC::write_byte(uint8_t txd) {
//    char i;
//    for (i = 0; i < 8; i++){
//        if ((txd << i) & 0x80) {
//            send_1_bit();
//        } else {
//            send_0_bit();
//        }
//    }
    uint8_t t;
    set_sda_output_mode();
    set_SCL(low);//拉低时钟开始数据传输
    for (t = 0; t < 8; t++) {
        if (txd & 0x80){
            set_SDA(high);
        }else{
            set_SDA(low);
        }
        txd <<= 1;
        DELAY(DELAY_TIME);
        set_SCL(high);
        DELAY(DELAY_TIME);
        set_SCL(low);
        DELAY(DELAY_TIME);
    }
    set_SCL(low);
    DELAY(DELAY_TIME);
    set_SDA(high);	//释放IIC_SDA总线，使得从设备可以发送ACK!!!!!!!!!!!!!!!!
    DELAY(DELAY_TIME);
}

//产生ACK应答
void IIC::send_ack(){
    //new
    set_SCL(low);
    set_sda_output_mode();
    //old
    set_SDA(low);
    DELAY(DELAY_TIME);
    set_SCL(high);
    DELAY(DELAY_TIME);
    set_SCL(low);
    DELAY(DELAY_TIME);
    set_SDA(high);
    DELAY(DELAY_TIME);
}

//不产生ACK应答
void IIC::not_send_ack() {
    //new
    set_SCL(low);
    set_sda_output_mode();
    //old
    set_SDA(high);
    DELAY(DELAY_TIME);
    set_SCL(high);
    DELAY(DELAY_TIME);
    set_SCL(low);
    DELAY(DELAY_TIME);
//    set_SDA(low);
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC::read_byte(uint8_t ack) {
    uint8_t i, receive = 0;
    for (i = 0; i < 8; i++) {
//        set_SDA(high);
//        DELAY(DELAY_TIME);
        set_SCL(high);
        DELAY(DELAY_TIME);
        receive <<= 1;
        if(read_SDA()){
//            DELAY(DELAY_TIME);
            receive |= 0x01;
        }
        set_SCL(low);
        DELAY(DELAY_TIME);
    }
    if (!ack){
        not_send_ack();
    }else{
        send_ack();
    }
    return receive;
}


uint8_t IIC::ReadI2CByte() {
    uint8_t b = 0, i;
    set_sda_input_mode();//SDA设置为输入
    bool f0 = false;
    for (i = 0; i < 8; i++) {
        set_SCL(low);
        DELAY(DELAY_TIME);
        set_SCL(high);
        DELAY(DELAY_TIME/2);
        b<<=1;
        if(read_SDA()){
            b++;
        }
        DELAY(DELAY_TIME/2);
//        old
//        set_SDA(high);
//        DELAY(DELAY_TIME);
//        set_SCL(high);
//        DELAY(DELAY_TIME);
//        f0 = read_SDA();
//        DELAY(DELAY_TIME);
//        set_SCL(low);
//        if (f0 == true) {
//            b = b << 1;
//            b = b | 0x01;
//        } else {
//            b = b << 1;
//        }
    }
    set_SDA(high);
    return b;
}


//----Read One Byte of Data,Data from SLAVER to the MASTER----
uint8_t IIC::read_byte_from_addr(uint8_t addr) {
    bool acktemp = true;
    uint8_t mydata;

    IIC_Start(); //IIC START
    write_byte((0x6D << 1) + 0);//IIC WRITE operation, SLAVER address bit: 0x6D
    is_receive_ack();//check the SLAVER
    write_byte(addr);
    is_receive_ack();//check the SLAVER

    IIC_Start(); //IIC START
    write_byte((0x6D << 1) + 1);//IIC WRITE operation, SLAVER address bit: 0x6D
    is_receive_ack();//check the SLAVER
    mydata = ReadI2CByte();
//    mydata = read_byte(0);
    IIC_Stop(); //IIC STOP
    return mydata;
}



//----write One Byte of Data,Data from MASTER to the SLAVER----
//Write "thedata" to the SLAVER's address of "addr"
void IIC::write_byte_to_addr(uint8_t addr, uint8_t byte) {
    IIC_Start();
    write_byte((0x6D << 1) + 0);//IIC WRITE operation, SLAVER address bit: 0x6D
    is_receive_ack();//check the SLAVER
    write_byte(addr);
    is_receive_ack();//check the SLAVER
    write_byte(byte);
    is_receive_ack();//check the SLAVER
    IIC_Stop(); //IIC STOP
}

