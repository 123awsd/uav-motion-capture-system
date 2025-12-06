#ifndef __BODY_CONTROL_H__
#define __BODY_CONTROL_H__
#include "main.h"


void Body_Balance_PID_Init(void);
void Get_Body_Motor_Duty(uint16_t* D_M1,uint16_t* D_M2,uint16_t* D_M3,uint16_t* D_M4);
void Body_Balance_Col_Usart_Show(void);//串口调试

void Body_Balance_PID_Change_NRF(void);//遥控器改pid
void Body_Balance_PID_Change_Usart(uint8_t * data ,uint8_t Num);//串口改pid
void Body_Balance_PID_Isum_Clear(void);

void Mode_Position_600Hz(float X_EXP, float Y_EXP, float Z_EXP,float Px, float Py, float Pz,float Yaw_w_EXP, uint16_t Basic_Speed);//定点模式
void Mode_Attitude_600Hz(float Roll_EXP, float Pitch_EXP,float Yaw_w_EXP, uint16_t Basic_Speed);//自稳模式


void Body_Rate_Control(float Roll_w_EXP, float Pitch_w_EXP, float Yaw_w_EXP, uint16_t Basic_Speed,int16_t *pM1, int16_t *pM2, int16_t *pM3, int16_t *pM4);//角速度环
void Body_Attitude_Control(float Roll_EXP, float Pitch_EXP, float Yaw_Rate_EXP, float Basic_Speed,float *pRoll_Rate_EXP, float *pPitch_Rate_EXP, float *pYaw_Rate_EXP);//角度环
void Body_Position_Control(float X_EXP, float Y_EXP, float Z_EXP,float Px,    float Py,    float Pz,float *Roll_EXP, float *Pitch_EXP, uint16_t *Basic_Speed);//位置环

#endif

