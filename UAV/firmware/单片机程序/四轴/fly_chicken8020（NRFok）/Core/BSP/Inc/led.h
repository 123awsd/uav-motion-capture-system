#ifndef __LED_H__
#define __LED_H__

#include "main.h"
#define   LED1_PORT	    GPIOA
#define   LED1_PIN	    GPIO_PIN_4
#define   LED2_PORT	    GPIOB
#define   LED2_PIN	    GPIO_PIN_10
#define   LED3_PORT	    GPIOB
#define   LED3_PIN	    GPIO_PIN_11
#define   LED4_PORT	    GPIOB
#define   LED4_PIN	    GPIO_PIN_9


#define   LED1_ON       HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
#define   LED1_OFF       HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
#define   LED1_TOGGLE    HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
#define   LED2_ON       HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
#define   LED2_OFF       HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
#define   LED2_TOGGLE    HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN);
#define   LED3_ON       HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_RESET);
#define   LED3_OFF       HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET);
#define   LED3_TOGGLE    HAL_GPIO_TogglePin(LED3_PORT, LED3_PIN);
#define   LED4_ON       HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, GPIO_PIN_RESET);
#define   LED4_OFF       HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, GPIO_PIN_SET);
#define   LED4_TOGGLE    HAL_GPIO_TogglePin(LED4_PORT, LED4_PIN);




void LED_Init(void);
void LED_Disp(uint8_t led_num);


#endif

