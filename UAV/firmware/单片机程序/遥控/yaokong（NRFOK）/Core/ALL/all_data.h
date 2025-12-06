#ifndef __ALL_DATA_H__
#define __ALL_DATA_H__

#include "main.h"
#define u8		uint8_t
#define u16		uint16_t
#define uint	uint32_t

typedef enum{
RX_DATA_OK =1,  //飞机接收到数据（不代表数据准确）
NOT_RX_DATA  ,   //飞机未接收到数据
RX_DATA_TRUE ,  //飞机接收到数据并且数据正确
	
}BODY_RX_STATE;//接收状态

//*************************//mpu6050解算的姿态角和角速度计数据
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

//*************************//遥控器控制指令
typedef struct{

	int16_t CH_R_QH;//右摇杆前后舵量
	int16_t CH_R_ZY;//
	int16_t CH_L_QH;
	int16_t CH_L_ZY;

}YK_CH_STR;//遥控器通道值	

//************************************************无线模块发送接收数据*************************//
typedef struct 
{
	u8     						JiaoYan_Hand;
	YK_CH_STR                   YK_CH;//遥控器指令
	bool                        All_Stop;//急停
	u8                          Fly_mode;
		//pid调试	
	float                       PID_Data[3];//对应kp,Ki,Kd;
	u8                         PID_ID;//pid调试的ID
	u8     						JiaoYan_Tail;

}NRF_ALL_SEND_DATA;//通过无线模块发送的所有信息
//************************************************通过无线模块接收的所有信息*************************//
typedef struct 
{
	u8     						JiaoYan_Hand;
	EULER_ANGLE_DATA_STR_short 		Eular_Angle;//欧拉角
	GYRO_DATA_STR_short              Gyro; // 角速度
	u8                          Body_Rx_State;// 
	u8                          Power_V;//电池电压 扩大50倍节约传输空间
	short
		Altitude;//扩大100倍节约传输空间
	u8     						JiaoYan_Tail;
	
}NRF_ALL_Receive_DATA;







//oled
extern u8g2_t u8g2;


//无线模块
extern u8 NRF_Rx_Buff[32];//发送数组
extern u8 NRF_Tx_Buff[32];//接收数组
extern NRF_ALL_SEND_DATA    NRF_Send_Data;//发送的数据
extern NRF_ALL_Receive_DATA  NRF_Receive_Data;//接收的数据
extern u8 YK_NRF_Rx_State ;//@


//mpu6050解算的姿态角,
extern EULER_ANGLE_DATA_STR   Body_Euler_Angle;//机体欧拉角
extern GYRO_DATA_STR          Body_Gyro;//机体角速度数据

//串口数据缓存
extern u8 USART_1_Buff[28];
extern u8 USART_1_Rdata[28];

//电压数据
 extern float Body_Power_V ;//机体电压
 //高度
 extern float Body_Altitude ;//高度
#endif
