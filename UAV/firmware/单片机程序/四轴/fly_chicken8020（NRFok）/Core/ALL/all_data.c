#include "all_data.h"
#include "main.h"



//无线模块  
u8 NRF_Rx_Buff[32];//发送数组
u8 NRF_Tx_Buff[32];//接收数组
NRF_ALL_SEND_DATA     NRF_Send_Data;
NRF_ALL_Receive_DATA  NRF_Receive_Data;
u8 NRF_Rx_State = 0;//1为接收成功，0为接收失败




//mpu6050解算的姿态角
EULER_ANGLE_DATA_STR   Body_Euler_Angle;//机体欧拉角
GYRO_DATA_STR          Body_Gyro;//机体角速度数据




//电机占空比
MOTOR_DUTY_STR  Motor_Duty ={0,0,0,0};


//机体控制信息
BODY_CONTROL_STR     Body_Control_Inf;


//串口接收
u8 USART_2_Buff[28];//缓冲区
u8 USART_2_Rdata[28];//收到的数据

u8 USART_3_Buff[28];//缓冲区
u8 USART_3_Rdata[28];//收到的数据


float Power_Voltage = 0 ;//电池电压



//测试
int Test_count = 0;



