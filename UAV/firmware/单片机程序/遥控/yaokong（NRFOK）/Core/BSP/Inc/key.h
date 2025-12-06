#ifndef __KEY_H__
#define __KEY_H__
#include "main.h"

#define					KEY_Side_L_GPIO_PORT			GPIOB    //侧面左按键
#define					KEY_Side_L_GPIO_PIN				GPIO_PIN_6
#define					KEY_Side_R_GPIO_PORT			GPIOA    //侧面右按键
#define					KEY_Side_R_GPIO_PIN				GPIO_PIN_12
#define					KEY_Front_U_GPIO_PORT			GPIOB    //正面上按键
#define					KEY_Front_U_GPIO_PIN			GPIO_PIN_5
#define					KEY_Front_D_GPIO_PORT			GPIOB    //正面下按键
#define					KEY_Front_D_GPIO_PIN			GPIO_PIN_13
#define					KEY_Front_R_GPIO_PORT			GPIOB    //正面右按键
#define					KEY_Front_R_GPIO_PIN			GPIO_PIN_6
#define					KEY_Front_L_GPIO_PORT			GPIOB    //正面左按键
#define					KEY_Front_L_GPIO_PIN			GPIO_PIN_7
#define					KEY_Rocker_L_GPIO_PORT		GPIOB    //摇杆左按键
#define					KEY_Rocker_L_GPIO_PIN			GPIO_PIN_14
#define					KEY_Rocker_R_GPIO_PORT		GPIOB    //摇杆右按键
#define					KEY_Rocker_R_GPIO_PIN			GPIO_PIN_4

#define                 Key_Down_Level					GPIO_PIN_RESET

#define 				Read_KEY_Side_L 						    HAL_GPIO_ReadPin(KEY_Side_L_GPIO_PORT,KEY_Side_L_GPIO_PIN) //读取侧面左按键
#define 				Read_Read_KEY_Side_R 						    HAL_GPIO_ReadPin(KEY_Side_R_GPIO_PORT, KEY_Side_R_GPIO_PIN) //读取侧面右按键
#define 				Read_KEY_Front_U 						HAL_GPIO_ReadPin(KEY_Front_U_GPIO_PORT, KEY_Front_U_GPIO_PIN) //读取正面上按键
#define 				Read_KEY_Front_D 						HAL_GPIO_ReadPin(KEY_Front_D_GPIO_PORT,KEY_Front_D_GPIO_PIN) //读取正面下按键
#define 				Read_KEY_Front_R 						HAL_GPIO_ReadPin(KEY_Front_R_GPIO_PORT, KEY_Front_R_GPIO_PIN) //读取正面右按键
#define 				Read_KEY_Front_L 						HAL_GPIO_ReadPin(KEY_Front_L_GPIO_PORT, KEY_Front_L_GPIO_PIN) //读取正面左按键
#define 				Read_KEY_Rocker_L 						HAL_GPIO_ReadPin(KEY_Rocker_L_GPIO_PORT, KEY_Rocker_L_GPIO_PIN) //读取摇杆左按键
#define 				Read_KEY_Rocker_R 						HAL_GPIO_ReadPin(KEY_Rocker_R_GPIO_PORT, KEY_Rocker_R_GPIO_PIN) //读取摇杆右按键

#define 				KeyLong_Time                800        //长按时间1
#define 				KeyLongLong_Time            1600        //长按时间2
typedef enum{
	NO_KEY=0,//没有按键按下
	KEY_Side_L,//测左
	KEY_Side_R,//侧右
	KEY_Front_U,//上
	KEY_Front_D,//下
	KEY_Front_R,//左
	KEY_Front_L,//右
	KEY_Rocker_R,//摇杆右
	KEY_Rocker_L,//摇杆左
}KEY_STATE_ENUM;
typedef enum {
	Down_Mode = 0,//按下
	Up_Mode,//松开
	Long_Mode,//长按
	LongLong_Mode//长长按
}Key_Get_Mode_ENUM;
void Key_Init(void);
void Key_Scan(void);
uint8_t  Get_Key_Val(uint8_t mode);
#endif


