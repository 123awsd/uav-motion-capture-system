#include "pid.h"
//////////////////////////////////////////////////////////////////////////////////
//函数名称：
//函数作用：初始化pid参数
//入口参数：
//返 回 值：
//说    明：
//作    者：WP
//时    间：2019.11.29	  
//////////////////////////////////////////////////////////////////////////////////
void PID_Init(float Kp,float Ki,float Kd,float Imax,float Omin,float Omax,PID_Str *PID)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->Imax = Imax;
	PID->Omin  = Omin;
	PID->Omax = Omax;
	PID->Err = PID->Err1 = PID->Err2 = PID->Exp = PID->Isum = PID ->Mea = PID->Out = 0;
	
}	

//////////////////////////////////////////////////////////////////////////////////
//函数名称：
//函数作用：pid复位
//入口参数：
//返 回 值：
//说    明：
//作    者：WP
//时    间：2019.11.29  
//////////////////////////////////////////////////////////////////////////////////
void PID_Reset(PID_Str *PID)
{
	PID->Kp = PID->Kp;
	PID->Ki = PID->Ki;
	PID->Kd = PID->Kd;
	PID->Imax = PID->Imax;
	PID->Omin  = PID->Omin;
	PID->Omax = PID->Omax;
	PID->Err = PID->Err1 = PID->Err2 = PID->Exp = PID->Isum = PID ->Mea = PID->Out = 0;
}

//////////////////////////////////////////////////////////////////////////////////
//函数名称：
//函数作用：位置式PID更新计算
//入口参数：expect：期望值   measure：测量值
//返 回 值：
//说    明：
//作    者：WP
//时    间：2019.11.29	  
//////////////////////////////////////////////////////////////////////////////////
void PID_Update_Pos(PID_Str *PID,float limit)
{
	static int time = 0;
	time++;
	if(time == 100) { time = 0;}
	
	
	PID->Err = PID->Exp - PID->Mea;
	
	PID->Isum += PID->Err;
	if(PID->Imax!=0.0f) PID->Isum = limitt(PID->Isum,-PID->Imax,PID->Imax);
	
	PID->Out = PID->Kp*PID->Err + PID->Ki*PID->Isum + PID->Kd*(PID->Err1 - PID->Err);
	
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);
	
//	if(ABS(PID->Err) < ABS(limit) && PID->Exp == 0 && time <= 5)
//	{
//	  PID->Isum=0;
//	  PID->Out=0;
//	}
	PID->Err1 = PID->Err;
	
	
	
}

void PID_Update_Pos_6020(PID_Str *PID,float limit)
{
	
	PID->Err = PID->Exp - PID->Mea;
	
	PID->Isum += PID->Err;
	if(PID->Imax!=0.0f) 
	PID->Isum = min(PID->Imax,PID->Isum);  //积分限幅为0则不限幅	
	PID->Out = PID->Kp*PID->Err + PID->Ki*PID->Isum + PID->Kd*(PID->Err1 - PID->Err);
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);
	
	if(ABS(PID->Out) < ABS(limit) && PID->Exp == 0)
	{
	  PID->Isum=0;
	  PID->Out=0;
	}
	
	PID->Err1 = PID->Err;
}


void PID_Update_Pos_angle_6020(PID_Str *PID)
{
	PID->Err = PID->Mea - PID->Exp;
	PID->Isum += PID->Err;
	if(PID->Imax!=0.0f)
	PID->Isum = min(PID->Imax,PID->Isum);  //积分限幅为0则不限幅	
	PID->Out = PID->Kp*PID->Err + PID->Ki*PID->Isum + PID->Kd*(PID->Err1 - PID->Err);
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);
	
	PID->Err1 = PID->Err;
}

void PID_Update_Pos_angle(PID_Str *PID)
{
	float err1,err2,err3,err;
	err1 = PID->Mea - PID->Exp;
	err2 = (PID->Mea+360) - PID->Exp;
	err3 = PID->Mea - (PID->Exp+360);
	err  = ((ABS(err1)<ABS(err2)) ? err1 : err2);
	PID->Err = ((ABS(err)<ABS(err3)) ? err : err3);
	
	PID->Isum += PID->Err;
//	if(PID->Imax!=0.0f) 
	PID->Isum = min(PID->Imax,PID->Isum);  //积分限幅为0则不限幅	
	PID->Out = PID->Kp*PID->Err + PID->Ki*PID->Isum + PID->Kd*(PID->Err1 - PID->Err);
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);
	
	PID->Err1 = PID->Err;
}

void PID_Update_Pos_Dlangle(PID_Str *PID)
{
	float err1,err2,err3,err;
	err1 = PID->Mea - PID->Exp;
	err2 = (PID->Mea+13107) - PID->Exp;
	err3 = PID->Mea - (PID->Exp+13107);
	err  = ((ABS(err1)<ABS(err2)) ? err1 : err2);
	PID->Err = ((ABS(err)<ABS(err3)) ? err : err3);
	
	PID->Isum += PID->Err;
//	if(PID->Imax!=0.0f) 
	PID->Isum = min(PID->Imax,PID->Isum);  //积分限幅为0则不限幅	
	PID->Out = PID->Kp*PID->Err + PID->Ki*PID->Isum + PID->Kd*(PID->Err1 - PID->Err);
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);
	
	PID->Err1 = PID->Err;
}

//////////////////////////////////////////////////////////////////////////////////
//函数名称：
//函数作用：增量式PID更新计算
//入口参数：expect：期望值   measure：测量值
//返 回 值：
//说    明：
//作    者：WP
//时    间：2019.11.29		  
//////////////////////////////////////////////////////////////////////////////////
void PID_Update_Inc(PID_Str *PID)
{
	PID->Err = PID->Mea - PID->Exp;
	
	PID->Out = PID->Kp*(PID->Err - PID->Err1) + PID->Ki*PID->Err + PID->Kd*(PID->Err - 2*PID->Err1 + PID->Err2);  //增量式PID标准公式
//	PID->Out = PID->Kp*PID->Err - PID->Ki*PID->Err1 + PID->Kd*PID->Err2;    //简化公式  需要不同的PID参数
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);
//	printf("%f\r\n",PID->Kp);
	PID->Err2 = PID->Err1;
	PID->Err1 = PID->Err;
}

void PID_Update_Inc_Angle(PID_Str *PID)
{
	float err1,err2;
	err1 = PID->Mea - PID->Exp;
	err2 = PID->Mea	- (PID->Exp+360);
	PID->Err = ABS(err1)<ABS(err2) ? err1 : err2;
	
	PID->Err = PID->Exp - PID->Mea;
	
	PID->Out = PID->Kp*(PID->Err - PID->Err1) + PID->Ki*PID->Err + PID->Kd*(PID->Err - 2*PID->Err1 + PID->Err2);  //增量式PID标准公式
//	PID->Out = PID->Kp*PID->Err - PID->Ki*PID->Err1 + PID->Kd*PID->Err2;    //简化公式  需要不同的PID参数
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);
	
	PID->Err2 = PID->Err1;
	PID->Err1 = PID->Err;
}

void PID_ParameterChange(float  Kp,float Ki,float Kd  ,PID_Str * PID)//pid参数修改
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
}

//飞机角度环专用pid,d为角速度
void PID_Update_Pos_Fly(PID_Str *PID,float Gyro)
{
//	static int time = 0;
//	time++;
//	if(time == 100) { time = 0;}
	
	
	PID->Err = PID->Exp - PID->Mea;
	
	PID->Isum += PID->Err;
	if(PID->Imax!=0.0f) PID->Isum = limitt(PID->Isum,-PID->Imax,PID->Imax);
//	if(PID->Imax < PID->Isum)PID->Isum = PID->Imax;
//	if(- PID->Imax > PID->Isum ) PID->Isum = -PID->Imax;
	PID->Out = PID->Kp*PID->Err + PID->Ki*PID->Isum + PID->Kd*(Gyro);
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);
	
}
////飞机角度环专用pid,d为角速度 航向角
void PID_Update_Pos_Fly_Z(PID_Str *PID,float Gyro)
{
	float err1,err2,err3,err;
	err1 = PID->Mea - PID->Exp;
	err2 = (PID->Mea+360) - PID->Exp;
	err3 = PID->Mea - (PID->Exp+360);
	err  = ((ABS(err1)<ABS(err2)) ? err1 : err2);
	PID->Err = ((ABS(err)<ABS(err3)) ? err : err3);
	
	PID->Isum += PID->Err;
	if(PID->Imax!=0.0f) PID->Isum = limitt(PID->Isum,-PID->Imax,PID->Imax);					
	PID->Out = PID->Kp*PID->Err + PID->Ki*PID->Isum + PID->Kd*(Gyro);            
	PID->Out = limitt(PID->Out,PID->Omin,PID->Omax);                           
	
}




