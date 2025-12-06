#ifndef _PID_H_
#define _PID_H_
#include "main.h"



typedef struct{
	float Kp;    //pid参数
	float Ki;
	float Kd;
	
	float Err;   //误差
	float Err1;
	float Err2; 
	
	float Isum;  //积分
	
	float Exp;   //期望值  测量值  输出值
	float Mea;
	float Out;
	
	float Imax;  //积分限幅
	
	float Omin;  //输出限幅
	float Omax;
}PID_Str;


void PID_Init(float Kp,float Ki,float Kd,float Imax,float Omin,float Omax,PID_Str *PID);
void PID_Reset(PID_Str *PID);
void PID_Update_Pos(PID_Str *PID,float limit);
void PID_Update_Pos_6020(PID_Str *PID,float limit);
void PID_Update_Inc(PID_Str *PID);
void PID_Update_Pos_angle_6020(PID_Str *PID);
void PID_Update_Pos_angle(PID_Str *PID);
void PID_Update_Pos_Dlangle(PID_Str *PID);
void PID_Update_Inc_Angle(PID_Str *PID);
void PID_ParameterChange(float  Kp,float Ki,float Kd  ,PID_Str * PID_temp);//pid参数修改
void PID_Update_Pos_Fly(PID_Str *PID,float Gyro);
void PID_Update_Pos_Fly_Z(PID_Str *PID,float Gyro);

#endif
