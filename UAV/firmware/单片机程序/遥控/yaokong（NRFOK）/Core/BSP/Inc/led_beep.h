#ifndef __LED_H__
#define __LED_H__

#include "main.h"


#define   LED1_PORT	    GPIOB
#define   LED1_PIN	    GPIO_PIN_9
#define   LED2_PORT	    GPIOB
#define   LED2_PIN	    GPIO_PIN_1

#define   LED1_ON       HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
#define   LED1_OFF       HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
#define   LED1_TOGGLE    HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
#define   LED2_ON       HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
#define   LED2_OFF       HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
#define   LED2_TOGGLE    HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN);

//


/* 有源蜂鸣器 引脚定义 */
#define BEEP_GPIO_PORT 				GPIOB
#define BEEP_GPIO_PIN 				GPIO_PIN_12


/* 有源蜂鸣器 开关控制 */

#define BEEP_ON  		HAL_GPIO_WritePin(BEEP_GPIO_PORT,BEEP_GPIO_PIN,GPIO_PIN_SET);
#define BEEP_OFF        HAL_GPIO_WritePin(BEEP_GPIO_PORT,BEEP_GPIO_PIN,GPIO_PIN_RESET);
#define BEEP_TOGGLE     HAL_GPIO_TogglePin(BEEP_GPIO_PORT,BEEP_GPIO_PIN);


/* 有源蜂鸣器 函数声明 */



#endif



