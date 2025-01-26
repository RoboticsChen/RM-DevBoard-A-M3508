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
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "usart.h"

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

#define NUM_POSITIONS 5
int target_angles[NUM_POSITIONS] = {0, -6400, 6400, 14000, -6400};  // 预设目标角度
int current_position_idx[2] = {0, 0};
// 当前角度目标索引，分别控制ID3和ID4电机
void PID_Parameter_Init(void) {
  // 1) 电机1、2速度PID的初始化
  PID_struct_init(&pid_spd[0], POSITION_PID, 8000, 8000, 1.5f, 0.1f, 0.3f, 200);
  PID_struct_init(&pid_spd[1], POSITION_PID, 8000, 8000, 1.5f, 0.1f, 0.3f, 200);

  // 2) 电机3、4速度环PID初始化
  PID_struct_init(&pid_spd[2], POSITION_PID, 2000, 2000, 1.0f, 0.0f, 0.3f, 200);
  PID_struct_init(&pid_spd[3], POSITION_PID, 2000, 2000, 1.0f, 0.0f, 0.3f, 200);

  // 3) 电机3、4位置环PID初始化（新增）
  //    位置环一般P值不要太大，D值可适当加大以减少超调振荡。I视具体情况而定
  PID_struct_init(&pid_pos[2], POSITION_PID, 2000, 2000, 1.0f, 0.0f, 0.3f, 200);
  PID_struct_init(&pid_pos[3], POSITION_PID, 2000, 2000, 1.0f, 0.0f, 0.3f, 200);
}

void stop_all_motors() {
  for (int i = 0; i < 4; i++) {
    set_spd[i] = 0;
  }
  set_moto_current(&hcan1, 0, 0, 0, 0);  // 发送停止指令
}

void handle_led(uint8_t number) {
  switch (number) {
    case 0:
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
      break;
    case 1:
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
      break;
    case 4:
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
      break;
    case 5:
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
      break;
    case 6:
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
      break;
    default:
      break;
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
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

  PID_Parameter_Init();
  HAL_Delay(100);

  int key_sta = 2, key_cnt;
  while (1) {
    switch (key_sta) {
      case 0:
        if (0 == HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
          key_sta = 1;
          HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
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
          HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
        }
        break;
    }

    if (key_cnt > 4) {
      key_cnt = 0;  // 计数值超过10后归零
    }
    float sp0, sp1, sp2, sp3, pos_out2, pos_out3;
    handle_led(key_cnt);
    if (key_cnt != 0) {
      sp0 = pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm, 5000);
      sp1 = pid_calc(&pid_spd[1], moto_chassis[1].speed_rpm, -5000);
    }
    // Corrected cascaded control for motors 2 and 3
    pos_out2 = pid_calc(&pid_pos[2], moto_chassis[2].total_angle, target_angles[key_cnt - 1]);
    sp2 = pid_calc(&pid_spd[2], moto_chassis[2].speed_rpm, pos_out2);  // Use position output as speed setpoint

    pos_out3 = pid_calc(&pid_pos[3], moto_chassis[3].total_angle, target_angles[key_cnt - 1]);
    sp3 = pid_calc(&pid_spd[3], moto_chassis[3].speed_rpm, pos_out3);  // Use position output as speed setpoint

    set_moto_current(&hcan1, (int16_t)sp0, (int16_t)sp1, (int16_t)sp2, (int16_t)sp3);
    HAL_Delay(1);

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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
