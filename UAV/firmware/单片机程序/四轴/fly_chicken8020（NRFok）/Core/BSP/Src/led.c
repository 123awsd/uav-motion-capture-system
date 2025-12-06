#include "led.h"


/**
  * @brief  led控制
  * @param  led_num：八位二进制数，0~3位分别对应LED1~4
  * @retval none
  */

void LED_Disp(uint8_t led_num)
{
	//全灭
	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, GPIO_PIN_SET);
	
	
	if(led_num & 0x01 )
	{
		HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
	}
	if(led_num & (0x01<<1) )
	{
		HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
	}
	if(led_num & (0x01<<2) )
	{
		HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_RESET);
	}
	if(led_num & (0x01<<3) )
	{
		HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, GPIO_PIN_RESET);
	}
	
	
}

