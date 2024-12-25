//
// Created by Henson on 2024-03-24.
//

#include "drv_key.h"
#include "main.h"

bool is_key_down(){
    if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET){
        return true;
    }else{
        return false;
    }
}