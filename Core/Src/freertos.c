/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "iwdg.h"
#include "user_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  bool is_enable;
  uint8_t len;
  uint8_t *buff;
  uint32_t stdid;
} can_tx_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t global_working_time = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for kinematics */
osThreadId_t kinematicsHandle;
const osThreadAttr_t kinematics_attributes = {
  .name = "kinematics",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for rcReceive */
osThreadId_t rcReceiveHandle;
const osThreadAttr_t rcReceive_attributes = {
  .name = "rcReceive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for communication */
osThreadId_t communicationHandle;
const osThreadAttr_t communication_attributes = {
  .name = "communication",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for armRollIMU */
osThreadId_t armRollIMUHandle;
const osThreadAttr_t armRollIMU_attributes = {
  .name = "armRollIMU",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can1Send */
osThreadId_t can1SendHandle;
const osThreadAttr_t can1Send_attributes = {
  .name = "can1Send",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can2Send */
osThreadId_t can2SendHandle;
const osThreadAttr_t can2Send_attributes = {
  .name = "can2Send",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for autoCtrl */
osThreadId_t autoCtrlHandle;
const osThreadAttr_t autoCtrl_attributes = {
  .name = "autoCtrl",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for motor */
osThreadId_t motorHandle;
const osThreadAttr_t motor_attributes = {
  .name = "motor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for test */
osThreadId_t testHandle;
const osThreadAttr_t test_attributes = {
  .name = "test",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for extendCtrl */
osThreadId_t extendCtrlHandle;
const osThreadAttr_t extendCtrl_attributes = {
  .name = "extendCtrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for slideCtrl */
osThreadId_t slideCtrlHandle;
const osThreadAttr_t slideCtrl_attributes = {
  .name = "slideCtrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for upliftCtrl */
osThreadId_t upliftCtrlHandle;
const osThreadAttr_t upliftCtrl_attributes = {
  .name = "upliftCtrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for armYawCtrl */
osThreadId_t armYawCtrlHandle;
const osThreadAttr_t armYawCtrl_attributes = {
  .name = "armYawCtrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for armRollCtrl */
osThreadId_t armRollCtrlHandle;
const osThreadAttr_t armRollCtrl_attributes = {
  .name = "armRollCtrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for yX2Ctrl */
osThreadId_t yX2CtrlHandle;
const osThreadAttr_t yX2Ctrl_attributes = {
  .name = "yX2Ctrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for djiMotorService */
osThreadId_t djiMotorServiceHandle;
const osThreadAttr_t djiMotorService_attributes = {
  .name = "djiMotorService",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for XGZP6847 */
osThreadId_t XGZP6847Handle;
const osThreadAttr_t XGZP6847_attributes = {
  .name = "XGZP6847",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for armPitchCtrl */
osThreadId_t armPitchCtrlHandle;
const osThreadAttr_t armPitchCtrl_attributes = {
  .name = "armPitchCtrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rc_control_Task */
osThreadId_t rc_control_TaskHandle;
const osThreadAttr_t rc_control_Task_attributes = {
  .name = "rc_control_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can1SendQueue */
osMessageQueueId_t can1SendQueueHandle;
const osMessageQueueAttr_t can1SendQueue_attributes = {
  .name = "can1SendQueue"
};
/* Definitions for can2SendQueue */
osMessageQueueId_t can2SendQueueHandle;
const osMessageQueueAttr_t can2SendQueue_attributes = {
  .name = "can2SendQueue"
};
/* Definitions for dr16UpdateBinarySem */
osSemaphoreId_t dr16UpdateBinarySemHandle;
const osSemaphoreAttr_t dr16UpdateBinarySem_attributes = {
  .name = "dr16UpdateBinarySem"
};
/* Definitions for CommCanBinarySem */
osSemaphoreId_t CommCanBinarySemHandle;
const osSemaphoreAttr_t CommCanBinarySem_attributes = {
  .name = "CommCanBinarySem"
};
/* Definitions for upliftLeftBinarySem */
osSemaphoreId_t upliftLeftBinarySemHandle;
const osSemaphoreAttr_t upliftLeftBinarySem_attributes = {
  .name = "upliftLeftBinarySem"
};
/* Definitions for upliftRightBinarySem */
osSemaphoreId_t upliftRightBinarySemHandle;
const osSemaphoreAttr_t upliftRightBinarySem_attributes = {
  .name = "upliftRightBinarySem"
};
/* Definitions for extendLeftBinarySem */
osSemaphoreId_t extendLeftBinarySemHandle;
const osSemaphoreAttr_t extendLeftBinarySem_attributes = {
  .name = "extendLeftBinarySem"
};
/* Definitions for extendRightBinarySem */
osSemaphoreId_t extendRightBinarySemHandle;
const osSemaphoreAttr_t extendRightBinarySem_attributes = {
  .name = "extendRightBinarySem"
};
/* Definitions for slideBinarySem */
osSemaphoreId_t slideBinarySemHandle;
const osSemaphoreAttr_t slideBinarySem_attributes = {
  .name = "slideBinarySem"
};
/* Definitions for armYawBinarySem */
osSemaphoreId_t armYawBinarySemHandle;
const osSemaphoreAttr_t armYawBinarySem_attributes = {
  .name = "armYawBinarySem"
};
/* Definitions for yawExtendBinarySem */
osSemaphoreId_t yawExtendBinarySemHandle;
const osSemaphoreAttr_t yawExtendBinarySem_attributes = {
  .name = "yawExtendBinarySem"
};
/* Definitions for pitchRollUpBinarySem */
osSemaphoreId_t pitchRollUpBinarySemHandle;
const osSemaphoreAttr_t pitchRollUpBinarySem_attributes = {
  .name = "pitchRollUpBinarySem"
};
/* Definitions for pitchRollDownBinarySem */
osSemaphoreId_t pitchRollDownBinarySemHandle;
const osSemaphoreAttr_t pitchRollDownBinarySem_attributes = {
  .name = "pitchRollDownBinarySem"
};
/* Definitions for cameraYawBinarySem */
osSemaphoreId_t cameraYawBinarySemHandle;
const osSemaphoreAttr_t cameraYawBinarySem_attributes = {
  .name = "cameraYawBinarySem"
};
/* Definitions for armRollImuBinarySem */
osSemaphoreId_t armRollImuBinarySemHandle;
const osSemaphoreAttr_t armRollImuBinarySem_attributes = {
  .name = "armRollImuBinarySem"
};
/* Definitions for armRollBinarySem */
osSemaphoreId_t armRollBinarySemHandle;
const osSemaphoreAttr_t armRollBinarySem_attributes = {
  .name = "armRollBinarySem"
};
/* Definitions for armYawDMBinarySem */
osSemaphoreId_t armYawDMBinarySemHandle;
const osSemaphoreAttr_t armYawDMBinarySem_attributes = {
  .name = "armYawDMBinarySem"
};
/* Definitions for armPitchDMBinarySem */
osSemaphoreId_t armPitchDMBinarySemHandle;
const osSemaphoreAttr_t armPitchDMBinarySem_attributes = {
  .name = "armPitchDMBinarySem"
};
/* Definitions for USBUpdateBinarySem */
osSemaphoreId_t USBUpdateBinarySemHandle;
const osSemaphoreAttr_t USBUpdateBinarySem_attributes = {
  .name = "USBUpdateBinarySem"
};
/* Definitions for can1_tx_cnt_sem */
osSemaphoreId_t can1_tx_cnt_semHandle;
const osSemaphoreAttr_t can1_tx_cnt_sem_attributes = {
  .name = "can1_tx_cnt_sem"
};
/* Definitions for can2_tx_cnt_sem */
osSemaphoreId_t can2_tx_cnt_semHandle;
const osSemaphoreAttr_t can2_tx_cnt_sem_attributes = {
  .name = "can2_tx_cnt_sem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void kinematicsTask(void *argument);
void rcReceiveTask(void *argument);
void communicationTask(void *argument);
void armRollIMUTask(void *argument);
void can1SendTask(void *argument);
void can2SendTask(void *argument);
void autoCtrlTask(void *argument);
void motorTask(void *argument);
void testTask(void *argument);
void extendCtrlTask(void *argument);
void slideCtrlTask(void *argument);
void upliftCtrlTask(void *argument);
void armYawCtrlTask(void *argument);
void armRollCtrlTask(void *argument);
void yX2CtrlTask(void *argument);
void djiMotorServiceTask(void *argument);
void XGZP6847Task(void *argument);
void armPitchCtrlTask(void *argument);
void RC_Control_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of dr16UpdateBinarySem */
  dr16UpdateBinarySemHandle = osSemaphoreNew(1, 1, &dr16UpdateBinarySem_attributes);

  /* creation of CommCanBinarySem */
  CommCanBinarySemHandle = osSemaphoreNew(1, 1, &CommCanBinarySem_attributes);

  /* creation of upliftLeftBinarySem */
  upliftLeftBinarySemHandle = osSemaphoreNew(1, 1, &upliftLeftBinarySem_attributes);

  /* creation of upliftRightBinarySem */
  upliftRightBinarySemHandle = osSemaphoreNew(1, 1, &upliftRightBinarySem_attributes);

  /* creation of extendLeftBinarySem */
  extendLeftBinarySemHandle = osSemaphoreNew(1, 1, &extendLeftBinarySem_attributes);

  /* creation of extendRightBinarySem */
  extendRightBinarySemHandle = osSemaphoreNew(1, 1, &extendRightBinarySem_attributes);

  /* creation of slideBinarySem */
  slideBinarySemHandle = osSemaphoreNew(1, 1, &slideBinarySem_attributes);

  /* creation of armYawBinarySem */
  armYawBinarySemHandle = osSemaphoreNew(1, 1, &armYawBinarySem_attributes);

  /* creation of yawExtendBinarySem */
  yawExtendBinarySemHandle = osSemaphoreNew(1, 1, &yawExtendBinarySem_attributes);

  /* creation of pitchRollUpBinarySem */
  pitchRollUpBinarySemHandle = osSemaphoreNew(1, 1, &pitchRollUpBinarySem_attributes);

  /* creation of pitchRollDownBinarySem */
  pitchRollDownBinarySemHandle = osSemaphoreNew(1, 1, &pitchRollDownBinarySem_attributes);

  /* creation of cameraYawBinarySem */
  cameraYawBinarySemHandle = osSemaphoreNew(1, 1, &cameraYawBinarySem_attributes);

  /* creation of armRollImuBinarySem */
  armRollImuBinarySemHandle = osSemaphoreNew(1, 1, &armRollImuBinarySem_attributes);

  /* creation of armRollBinarySem */
  armRollBinarySemHandle = osSemaphoreNew(1, 1, &armRollBinarySem_attributes);

  /* creation of armYawDMBinarySem */
  armYawDMBinarySemHandle = osSemaphoreNew(1, 1, &armYawDMBinarySem_attributes);

  /* creation of armPitchDMBinarySem */
  armPitchDMBinarySemHandle = osSemaphoreNew(1, 1, &armPitchDMBinarySem_attributes);

  /* creation of USBUpdateBinarySem */
  USBUpdateBinarySemHandle = osSemaphoreNew(1, 0, &USBUpdateBinarySem_attributes);

  /* creation of can1_tx_cnt_sem */
  can1_tx_cnt_semHandle = osSemaphoreNew(3, 3, &can1_tx_cnt_sem_attributes);

  /* creation of can2_tx_cnt_sem */
  can2_tx_cnt_semHandle = osSemaphoreNew(3, 3, &can2_tx_cnt_sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of can1SendQueue */
  can1SendQueueHandle = osMessageQueueNew (32, sizeof(can_tx_t), &can1SendQueue_attributes);

  /* creation of can2SendQueue */
  can2SendQueueHandle = osMessageQueueNew (32, sizeof(can_tx_t), &can2SendQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of kinematics */
  kinematicsHandle = osThreadNew(kinematicsTask, NULL, &kinematics_attributes);

  /* creation of rcReceive */
  rcReceiveHandle = osThreadNew(rcReceiveTask, NULL, &rcReceive_attributes);

  /* creation of communication */
  communicationHandle = osThreadNew(communicationTask, NULL, &communication_attributes);

  /* creation of armRollIMU */
  armRollIMUHandle = osThreadNew(armRollIMUTask, NULL, &armRollIMU_attributes);

  /* creation of can1Send */
  can1SendHandle = osThreadNew(can1SendTask, NULL, &can1Send_attributes);

  /* creation of can2Send */
  can2SendHandle = osThreadNew(can2SendTask, NULL, &can2Send_attributes);

  /* creation of autoCtrl */
  autoCtrlHandle = osThreadNew(autoCtrlTask, NULL, &autoCtrl_attributes);

  /* creation of motor */
  motorHandle = osThreadNew(motorTask, NULL, &motor_attributes);

  /* creation of test */
  testHandle = osThreadNew(testTask, NULL, &test_attributes);

  /* creation of extendCtrl */
  extendCtrlHandle = osThreadNew(extendCtrlTask, NULL, &extendCtrl_attributes);

  /* creation of slideCtrl */
  slideCtrlHandle = osThreadNew(slideCtrlTask, NULL, &slideCtrl_attributes);

  /* creation of upliftCtrl */
  upliftCtrlHandle = osThreadNew(upliftCtrlTask, NULL, &upliftCtrl_attributes);

  /* creation of armYawCtrl */
  armYawCtrlHandle = osThreadNew(armYawCtrlTask, NULL, &armYawCtrl_attributes);

  /* creation of armRollCtrl */
  armRollCtrlHandle = osThreadNew(armRollCtrlTask, NULL, &armRollCtrl_attributes);

  /* creation of yX2Ctrl */
  yX2CtrlHandle = osThreadNew(yX2CtrlTask, NULL, &yX2Ctrl_attributes);

  /* creation of djiMotorService */
  djiMotorServiceHandle = osThreadNew(djiMotorServiceTask, NULL, &djiMotorService_attributes);

  /* creation of XGZP6847 */
  XGZP6847Handle = osThreadNew(XGZP6847Task, NULL, &XGZP6847_attributes);

  /* creation of armPitchCtrl */
  armPitchCtrlHandle = osThreadNew(armPitchCtrlTask, NULL, &armPitchCtrl_attributes);

  /* creation of rc_control_Task */
  rc_control_TaskHandle = osThreadNew(RC_Control_Task, NULL, &rc_control_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    global_working_time = HAL_GetTick();
    HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
    HAL_IWDG_Refresh(&hiwdg);//(625-1+1)*64/32000s
    osDelay(200);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_kinematicsTask */
/**
* @brief Function implementing the kinematics thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_kinematicsTask */
__weak void kinematicsTask(void *argument)
{
  /* USER CODE BEGIN kinematicsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END kinematicsTask */
}

/* USER CODE BEGIN Header_rcReceiveTask */
/**
* @brief Function implementing the rcReceive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rcReceiveTask */
__weak void rcReceiveTask(void *argument)
{
  /* USER CODE BEGIN rcReceiveTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rcReceiveTask */
}

/* USER CODE BEGIN Header_communicationTask */
/**
* @brief Function implementing the communication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_communicationTask */
__weak void communicationTask(void *argument)
{
  /* USER CODE BEGIN communicationTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END communicationTask */
}

/* USER CODE BEGIN Header_armRollIMUTask */
/**
* @brief Function implementing the armRollIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_armRollIMUTask */
__weak void armRollIMUTask(void *argument)
{
  /* USER CODE BEGIN armRollIMUTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END armRollIMUTask */
}

/* USER CODE BEGIN Header_can1SendTask */
/**
* @brief Function implementing the can1Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can1SendTask */
__weak void can1SendTask(void *argument)
{
  /* USER CODE BEGIN can1SendTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can1SendTask */
}

/* USER CODE BEGIN Header_can2SendTask */
/**
* @brief Function implementing the can2Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can2SendTask */
__weak void can2SendTask(void *argument)
{
  /* USER CODE BEGIN can2SendTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can2SendTask */
}

/* USER CODE BEGIN Header_autoCtrlTask */
/**
* @brief Function implementing the autoCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_autoCtrlTask */
__weak void autoCtrlTask(void *argument)
{
  /* USER CODE BEGIN autoCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END autoCtrlTask */
}

/* USER CODE BEGIN Header_motorTask */
/**
* @brief Function implementing the motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorTask */
__weak void motorTask(void *argument)
{
  /* USER CODE BEGIN motorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motorTask */
}

/* USER CODE BEGIN Header_testTask */
/**
* @brief Function implementing the test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_testTask */
__weak void testTask(void *argument)
{
  /* USER CODE BEGIN testTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END testTask */
}

/* USER CODE BEGIN Header_extendCtrlTask */
/**
* @brief Function implementing the extendCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_extendCtrlTask */
__weak void extendCtrlTask(void *argument)
{
  /* USER CODE BEGIN extendCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END extendCtrlTask */
}

/* USER CODE BEGIN Header_slideCtrlTask */
/**
* @brief Function implementing the slideCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_slideCtrlTask */
__weak void slideCtrlTask(void *argument)
{
  /* USER CODE BEGIN slideCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END slideCtrlTask */
}

/* USER CODE BEGIN Header_upliftCtrlTask */
/**
* @brief Function implementing the upliftCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_upliftCtrlTask */
__weak void upliftCtrlTask(void *argument)
{
  /* USER CODE BEGIN upliftCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END upliftCtrlTask */
}

/* USER CODE BEGIN Header_armYawCtrlTask */
/**
* @brief Function implementing the armYawCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_armYawCtrlTask */
__weak void armYawCtrlTask(void *argument)
{
  /* USER CODE BEGIN armYawCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END armYawCtrlTask */
}

/* USER CODE BEGIN Header_armRollCtrlTask */
/**
* @brief Function implementing the armRollCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_armRollCtrlTask */
__weak void armRollCtrlTask(void *argument)
{
  /* USER CODE BEGIN armRollCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END armRollCtrlTask */
}

/* USER CODE BEGIN Header_yX2CtrlTask */
/**
* @brief Function implementing the yX2Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_yX2CtrlTask */
__weak void yX2CtrlTask(void *argument)
{
  /* USER CODE BEGIN yX2CtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END yX2CtrlTask */
}

/* USER CODE BEGIN Header_djiMotorServiceTask */
/**
* @brief Function implementing the djiMotorService thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_djiMotorServiceTask */
__weak void djiMotorServiceTask(void *argument)
{
  /* USER CODE BEGIN djiMotorServiceTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END djiMotorServiceTask */
}

/* USER CODE BEGIN Header_XGZP6847Task */
/**
* @brief Function implementing the XGZP6847 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_XGZP6847Task */
__weak void XGZP6847Task(void *argument)
{
  /* USER CODE BEGIN XGZP6847Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END XGZP6847Task */
}

/* USER CODE BEGIN Header_armPitchCtrlTask */
/**
* @brief Function implementing the armPitchCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_armPitchCtrlTask */
__weak void armPitchCtrlTask(void *argument)
{
  /* USER CODE BEGIN armPitchCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END armPitchCtrlTask */
}

/* USER CODE BEGIN Header_RC_Control_Task */
/**
* @brief Function implementing the rc_control_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_Control_Task */
__weak void RC_Control_Task(void *argument)
{
  /* USER CODE BEGIN RC_Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RC_Control_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

