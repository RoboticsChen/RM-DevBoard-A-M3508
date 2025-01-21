/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "bsp_can.h"
#include "pid.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "gpio.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int set_spd[4] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define NUM_POSITIONS 4
int target_angles[NUM_POSITIONS] = {0, 900, 1800, 2700};  // 预设目标角度
int current_position_idx[2] = {0, 0};                     // 当前角度目标索引，分别控制ID3和ID4电机

void stop_all_motors() {
  for (int i = 0; i < 4; i++) {
    set_spd[i] = 0;
  }
  set_moto_current(&hcan1, 0, 0, 0, 0);  // 发送停止指令
}

void set_motor_position(CAN_HandleTypeDef* hcan, int motor_id, int target_angle) {
  // 设置目标位置的控制
  // 根据电机的实际情况和控制逻辑来计算控制信号。这里假设有一个位置控制的PID。
  // 目标角度已经通过target_angles数组传递。

  if (motor_id == 2 || motor_id == 3) {                      // 3号和4号电机进行位置控制
    int current_angle = moto_chassis[motor_id].total_angle;  // 获取当前角度
    int error = target_angle - current_angle;
    if (abs(error) < 100) {   // 位置控制的精度
      set_spd[motor_id] = 0;  // 当接近目标位置时停止
    } else {
      // 设置PID控制，调节电机转速使其到达目标位置
      set_spd[motor_id] = (error > 0) ? 100 : -100;  // 速度方向根据误差确定
    }
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  my_can_filter_init_recv_all(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  for (int i = 0; i < 4; i++) {
    PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000, 5.0f, 1.0f, 0.0f);  // 4 motos angular rate closeloop.
  }
  HAL_Delay(100);

  int key_sta = 2, key_cnt;
  while (1) {
    for (int i = 0; i < 4; i++) {
      pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
    }
    set_moto_current(&hcan1, pid_spd[0].pos_out, pid_spd[1].pos_out, pid_spd[2].pos_out, pid_spd[3].pos_out);
    switch (key_sta) {
      case 0:
        if (0 == HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
          key_sta = 1;
          HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        }
        break;
      case 1:
        if (0 == HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
          key_sta = 2;
          key_cnt++;
        } else {
          key_sta = 0;
        }
        break;
      case 2:
        if (0 != HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
          key_sta = 0;
          HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        }
        break;
    }

    if (key_cnt > 4) {
      key_cnt = 0;  // 计数值超过10后归零
    }

    switch (key_cnt) {  // 按键计数值在0-4之间循环
      case 0:
        stop_all_motors();
        break;
      case 1:
        // 状态1：电机1、2保持固定速度，电机3、4切换到目标角度
        set_spd[0] = set_spd[1] = 500;                    // 电机1、2速度为500
        set_motor_position(&hcan1, 2, target_angles[0]);  // 电机3设置目标角度0
        set_motor_position(&hcan1, 3, target_angles[0]);  // 电机4设置目标角度0
        break;
      case 2:
        // 状态2：电机1、2保持固定速度，电机3、4切换到目标角度
        set_spd[0] = set_spd[1] = 500;                    // 电机1、2速度为500
        set_motor_position(&hcan1, 2, target_angles[1]);  // 电机3设置目标角度1
        set_motor_position(&hcan1, 3, target_angles[1]);  // 电机4设置目标角度1
        break;
      case 3:
        // 状态3：电机1、2保持固定速度，电机3、4切换到目标角度
        set_spd[0] = set_spd[1] = 500;                    // 电机1、2速度为500
        set_motor_position(&hcan1, 2, target_angles[2]);  // 电机3设置目标角度2
        set_motor_position(&hcan1, 3, target_angles[2]);  // 电机4设置目标角度2
        break;
      case 4:
        // 状态4：电机1、2保持固定速度，电机3、4切换到目标角度
        set_spd[0] = set_spd[1] = 500;                    // 电机1、2速度为500
        set_motor_position(&hcan1, 2, target_angles[3]);  // 电机3设置目标角度3
        set_motor_position(&hcan1, 3, target_angles[3]);  // 电机4设置目标角度3
        break;
    }

    HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
