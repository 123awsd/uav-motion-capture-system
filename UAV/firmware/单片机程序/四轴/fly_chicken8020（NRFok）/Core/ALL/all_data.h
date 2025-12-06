#ifndef __ALL_DATA_H__
#define __ALL_DATA_H__

#include "main.h"
#define u8		uint8_t
#define u16		uint16_t
#define uint	uint32_t

#define   Motor_PWM_Duty_Max   7000  //pwm占空比最大值
//角度控制指令最大值
#define   Pitch_Commend_Max    30.0f //
#define   Roll_Commend_Max    30.0f //
#define   Yaw_W_Commend_Max    100.0f //角速度






//************************************mpu6050解算的姿态角和角速度计数据****************************//
typedef struct{

	float yaw;//航向角
	float pitch;//俯仰角
	float roll;//横滚
}EULER_ANGLE_DATA_STR;	
typedef struct{

	float gx;//航向角
	float gy;//俯仰角
	float gz;//横滚
}GYRO_DATA_STR;

typedef struct{

	short yaw;//航向角
	short pitch;//俯仰角
	short roll;//横滚
}EULER_ANGLE_DATA_STR_short;	//扩大80倍用short发送
typedef struct{

	short gx;//航向角
	short gy;//俯仰角
	short gz;//横滚
}GYRO_DATA_STR_short;//扩大80倍用short发送

//***********************************************遥控器控制指令****************************//
typedef struct{

	int16_t CH_R_QH;//右摇杆前后舵量
	int16_t CH_R_ZY;//
	int16_t CH_L_QH;
	int16_t CH_L_ZY;

}YK_CH_STR;//遥控原始通道值	


//************************************************无线模块发送接收数据*************************//
typedef struct 
{
	u8     						JiaoYan_Hand;
	EULER_ANGLE_DATA_STR_short 		Eular_Angle;//欧拉角
	GYRO_DATA_STR_short               Gyro; // 角速度
	u8                          Body_Rx_State;// 
	u8                          Power_V;//电池电压 扩大50倍节约传输空间
	short                         Altitude;//扩大100倍节约传输空间
	u8     						JiaoYan_Tail;
}NRF_ALL_SEND_DATA;//通过无线模块发送的所有信息

//************************************************无线模块发送接收数据*************************//
typedef struct 
{
	u8     						JiaoYan_Hand;
	YK_CH_STR                   YK_CH;//遥控器指令
	bool                        All_Stop;//急停
	u8                          Fly_mode;//飞行模式
		//pid调试	
	float                       PID_Data[3];//对应kp,Ki,Kd;
	u8                         PID_ID;//pid调试的ID
	u8     						JiaoYan_Tail;
}NRF_ALL_Receive_DATA;//通过无线模块接收的所有信息


//************************************************电机占空比********************************//
typedef struct {
u16       m1;
u16		  m2;
u16		  m3;
u16		  m4;

}MOTOR_DUTY_STR;


//************************************************机体控制
typedef struct{

	float Pitch;//Pitch控制
	float Roll;//
	float Yaw_w;//yaw角速度控制
	u16 Speed_Basic;

}ATTITUDE_CONTROL_STR;//飞机姿态控制
typedef struct{
	  float X_Pos;         // 当前目标 X 位置（单位：m）
    float Y_Pos;         // 当前目标 Y 位置（单位：m）
    float Z_Pos;         // 当前目标高度（单位：m）
	
	
} POSITION_CONTROL_STR;
//************************************************所有机体控制（后续添加高度，位置，速度控制）
typedef struct{

	ATTITUDE_CONTROL_STR   Attitude_Commend;//姿态控制指令

}BODY_CONTROL_STR;//飞机控制信息



//无线模块
extern u8 NRF_Rx_Buff[32];//发送数组
extern u8 NRF_Tx_Buff[32];//接收数组
extern NRF_ALL_SEND_DATA    NRF_Send_Data;//发送的数据
extern NRF_ALL_Receive_DATA  NRF_Receive_Data;//接收的数据
extern u8 NRF_Rx_State ;//接收是否成功 true为成功


//mpu6050解算的姿态角,
extern EULER_ANGLE_DATA_STR   Body_Euler_Angle;//机体欧拉角
extern GYRO_DATA_STR          Body_Gyro;//机体角速度数据


//串口数据缓存
extern u8 USART_2_Buff[28];
extern u8 USART_2_Rdata[28];
extern u8 USART_3_Buff[28];//缓冲区
extern u8 USART_3_Rdata[28];//收到的数据
//电机占空比
extern MOTOR_DUTY_STR  Motor_Duty;

//机体控制信息
extern BODY_CONTROL_STR     Body_Control_Inf;//机体控制信息

//电池adc采样
extern float Power_Voltage ;//电池电压
//测试
extern int Test_count;//测试
#endif
