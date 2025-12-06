#include "all_data.h"
#include "main.h"

//OLED
u8g2_t u8g2;

//无线模块  
u8 NRF_Rx_Buff[32];//发送数组
u8 NRF_Tx_Buff[32];//接收数组
NRF_ALL_SEND_DATA    NRF_Send_Data;
NRF_ALL_Receive_DATA  NRF_Receive_Data;
u8 YK_NRF_Rx_State = 0;//@BODY_RX_STATE 2为数据准确 1为接收成功，0为接收失败




//mpu6050解算的姿态角
EULER_ANGLE_DATA_STR   Body_Euler_Angle;//机体欧拉角
GYRO_DATA_STR          Body_Gyro;//机体角速度数据

//串口数据
u8 USART_1_Buff[28];
 u8 USART_1_Rdata[28];
 
 
 
 //电压数据 
 float Body_Power_V ;//机体电压
 
//高度
 float Body_Altitude ;//高度
 

//void 





