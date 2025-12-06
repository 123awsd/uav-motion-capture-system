/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
// 初始化IO，防止PB3不能使用作为LED，PB4用于频率输出
//// 注意：修改前需要确保AFIO配置正确
// AFIO->MAPR = 0X02000000; // 使能复用功能，关闭JTAG保留SWD，释放PB3和PB4端口，用于自定义功能（参考STM32F103手册第118页）


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim4);// 启动定时器4，用作普通定时
  HAL_TIM_Base_Start_IT(&htim2);//��ʱ��һ1ms�ж������������
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  Motor_Set_Duty(0,0,0,0);

  printf("hallo\r\n");
	
   // *************************** 任务初始化
	 HAL_Delay(2000);
	 ALL_Init();
	 HAL_Delay(1000);
   MY_Task_Begin();         // 任务开始
//	 Motor_Set_Duty(100,100,100,100);


//    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);// 电机1初始占空比0
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,800);//m2
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,800);//m3
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,800);//m4
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
//	int32_t temp, press;
//	float pitch, roll, Altitude1,yaw,Altitude2;
//	static MPU_ACC_GYRO_DATA_STR   mpu_data;//角速度加速度原始数据

  while (1)
  {
	  

	 	While_Loop();
	  HAL_Delay(500);
		
//		Imu_Euler_Angle_Update(50);// 获取当前欧拉角数据
//    Imu_Get_Euler_Angle_Data(&pitch, &roll, &yaw);  
//    printf("fffffffffffffffff  %.2f, %.2f, %.2f\r\n", pitch, roll, yaw);// 打印角度数据（保留2位小数）
//printf("fffffffffffffffff  %d, %d, %d\r\n", (int)pitch, (int)roll, (int)yaw);
	  
// mpu_data = My_Get_MPU_Data();
//    printf("【FFF.原始数据】ax=%.2f, ay=%.2f, az=%.2f | gx=%.2f, gy=%.2f, gz=%.2f\r\n",
//           mpu_data.ax, mpu_data.ay, mpu_data.az,
//           mpu_data.gx, mpu_data.gy, mpu_data.gz);
	  

		
//	  printf("t:%d\r\n",TIM4->CNT);
//temp  = bmp280_GetTemperature();   // 必须先调用，更新 t_fine
//press = bmp280_GetPressure();
//		press = bmp280_GetPressure();
//		Altitude1 = bmp280_GetAltitude();
//		Altitude2 = bmp280_GetRelativeAltitude();
//		printf("T = %.2f C, P = %d Pa   A1:%.2f  A2:%.2f\r\n  ", temp / 100.0f, press,Altitude1,Altitude2);

	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
