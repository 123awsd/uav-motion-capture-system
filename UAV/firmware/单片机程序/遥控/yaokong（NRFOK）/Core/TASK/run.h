
#ifndef __RUN_H__
#define __RUN_H__

#include "main.h"
#define RX_Mode  1
#define TX_Mode  0
typedef struct{

	float yaw;//航向角
	float pitch;//俯仰角
	float roll;//横滚
}EULER_ANGLE_STR;



void NFR_T_AND_R(void);
void Uasrt_Show(void);
void Rocker_Task(void);//摇杆任务
void Key_Task(void);//按键任务
void Beep_Task(void);//蜂鸣器任务
void ALL_Init(void);
void Oled_Show(void);
void Body_PID_Change_Usart(uint8_t * data ,uint8_t Num);//PID调参    格式：  #0,0,0@     对应Kp,Ki,Kd
#endif

