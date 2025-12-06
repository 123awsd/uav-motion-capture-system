
#ifndef __RUN_H__
#define __RUN_H__

#include "main.h"
//无线接收define
//#define RX_Mode  1
//#define TX_Mode  0
void ALL_Init(void);
void Eular_Angle_Update(void);
void Drone_Control_Task(void);//无人机控制


void Uasrt_Show(void);

void Drone_PID_Parameter_Tuning_600HZ(uint8_t PID_Mode);//PID调参控制
void PID_Change(void);//pid调参(放到while_loop)
void Drone_PID_Parameter_Usart_show(void);//PID调参曲线打印
float Get_Body_Power(void);
#endif

