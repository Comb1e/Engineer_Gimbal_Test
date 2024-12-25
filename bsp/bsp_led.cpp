//
// Created by Henson on 2024-03-27.
//

#include "bsp_led.h"
#include "main.h"

void set_blue_toggle() {
    HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
}
void set_blue_on() {
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
}
void set_blue_off() {
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
}

void set_red_toggle() {
    HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
}
void set_red_on() {
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
}
void set_red_off() {
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
}

void set_green_toggle() {
    HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
}
void set_green_on() {
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
}
void set_green_off() {
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
}

