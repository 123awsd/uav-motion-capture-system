#include "run.h"
#include "usart.h"

u8 page=2;//oled页码

//按键蜂鸣器提示
bool Key_BEEP_ENABLE = true;//使能蜂鸣器提示
u8 BEEP_Sound_Number; //蜂鸣器响次数

/////////////////////////////////////////////////////////////////////////串口**************************************************************************************
/**
 * @brief  串口发送NRF接收的数据到上位机（格式化输出）
 * @note   直接使用现有全局变量，无需额外定义
 * 
 * @接收格式说明（上位机需按此规则解析）：
 * 1. 帧结构：$UART,校验头,校验尾,yaw,pitch,roll,gx,gy,gz,高度,状态码,电压*校验和\r\n
 *    - 帧头：$UART, （固定5字节，标识帧开始）
 *    - 数据段：11个字段，用英文逗号','分隔
 *    - 校验和：*XX （XX为2位十六进制数，用于简单数据校验）
 *    - 帧尾：\r\n （回车换行，标识帧结束）
 * 
 * 2. 字段说明：
 *    序号 | 字段名       | 数据类型       | 单位    | 示例值
 *    1    | 校验头       | 十六进制（无0x）| -       | AA
 *    2    | 校验尾       | 十六进制（无0x）| -       | 55
 *    3    | 航向角yaw    | 浮点数（2位小数）| °       | 12.34
 *    4    | 俯仰角pitch  | 浮点数（2位小数）| °       | -5.67
 *    5    | 横滚角roll   | 浮点数（2位小数）| °       | 8.90
 *    6    | X轴角速度gx  | 浮点数（2位小数）| °/s     | 1.23
 *    7    | Y轴角速度gy  | 浮点数（2位小数）| °/s     | 4.56
 *    8    | Z轴角速度gz  | 浮点数（2位小数）| °/s     | -7.89
 *    9    | 当前高度     | 浮点数（2位小数）| m       | 10.11
 *    10   | 接收状态码   | 整数           | -       | 2（0=未接收，1=接收但不准确，2=接收准确，0xFF=未知）
 *    11   | 电池电压     | 浮点数（2位小数）| V       | 3.78
 * 
 * 3. 示例帧：
 *    $UART,AA,55,12.34,-5.67,8.90,1.23,4.56,-7.89,10.11,2,3.78*3C\r\n
 */
void UART_SendToUpperComputer(void) {
    // 转换接收状态为数字编码
    uint8_t state_code;
    switch(YK_NRF_Rx_State) {
        case RX_DATA_OK:      state_code = 1;  break;  
        case NOT_RX_DATA:     state_code = 0;  break;  
        case RX_DATA_TRUE:    state_code = 2;  break;  
        default:              state_code = 0xFF;break; 
    }

    // 计算校验和用的欧拉角整数（只算一次，同时用于校验和和发送）
    int yaw_int = (int)(Body_Euler_Angle.yaw * 100);    // 关键：用原始值计算，不依赖打印
    int pitch_int = (int)(Body_Euler_Angle.pitch * 100);
    int roll_int = (int)(Body_Euler_Angle.roll * 100);

    // 计算校验和（用上述整数，与发送的整数完全一致）
    uint8_t check_sum = 0;
    check_sum += NRF_Receive_Data.JiaoYan_Hand;
    check_sum += NRF_Receive_Data.JiaoYan_Tail;
    check_sum += (uint8_t)yaw_int;    // 直接用计算好的整数
    check_sum += (uint8_t)pitch_int;
    check_sum += (uint8_t)roll_int;
    check_sum += state_code;

    // 新帧格式：新增yaw_int/pitch_int/roll_int字段（第3-5位）
    printf("$UART,"                          // 帧头
           "%02X,"                           // 1. 校验头
           "%02X,"                           // 2. 校验尾
           "%d,"                             // 3. yaw整数（用于校验，如30188）
           "%d,"                             // 4. pitch整数（用于校验，如-1895）
           "%d,"                             // 5. roll整数（用于校验，如-1521）
           "%.2f,"                           // 6. 航向角yaw（显示用，如301.89）
           "%.2f,"                           // 7. 俯仰角pitch（显示用，如-18.95）
           "%.2f,"                           // 8. 横滚角roll（显示用，如-15.21）
           "%.2f,"                           // 9. X轴角速度gx
           "%.2f,"                           // 10. Y轴角速度gy
           "%.2f,"                           // 11. Z轴角速度gz
           "%.2f,"                           // 12. 当前高度
           "%d,"                             // 13. 接收状态码
           "%.2f"                            // 14. 电池电压
           "*%02X\r\n",                      // 校验和+帧尾
           
           // 对应变量（顺序与上述字段一致）
           NRF_Receive_Data.JiaoYan_Hand,
           NRF_Receive_Data.JiaoYan_Tail,
           yaw_int,          // 新增：校验用yaw整数
           pitch_int,        // 新增：校验用pitch整数
           roll_int,         // 新增：校验用roll整数
           Body_Euler_Angle.yaw,  // 显示用yaw（%.2f）
           Body_Euler_Angle.pitch,// 显示用pitch（%.2f）
           Body_Euler_Angle.roll, // 显示用roll（%.2f）
           Body_Gyro.gx,
           Body_Gyro.gy,
           Body_Gyro.gz,
           Body_Altitude,
           state_code,
           Body_Power_V,
           check_sum);
}
/**
 * @brief  带调试打印的串口发送函数（保留原调试信息，同时发送格式化数据）
 * @note   适用于调试阶段，既方便串口助手查看，又能被上位机解析
 */
void UART_Print_Show_State(void) {
    // 保留原有的调试打印（人类可读格式）
    printf("===== NRF接收数据信息 =====\r\n");
    printf("校验头: 0x%02X\r\n", NRF_Receive_Data.JiaoYan_Hand);
    printf("校验尾: 0x%02X\r\n", NRF_Receive_Data.JiaoYan_Tail);
    printf("\r\n");
    printf("【角度数据】\r\n");
    printf("航向角(yaw): %.2f°\r\n", Body_Euler_Angle.yaw);
    printf("俯仰角(pitch): %.2f°\r\n", Body_Euler_Angle.pitch);
    printf("横滚角(roll): %.2f°\r\n", Body_Euler_Angle.roll);
    printf("\r\n");
    printf("【角速度数据】\r\n");
    printf("X轴角速度(gx): %.2f°/s\r\n", Body_Gyro.gx);
    printf("Y轴角速度(gy): %.2f°/s\r\n", Body_Gyro.gy);
    printf("Z轴角速度(gz): %.2f°/s\r\n", Body_Gyro.gz);
    printf("\r\n");
    printf("【环境数据】\r\n");
    printf("当前高度: %.2fm\r\n", Body_Altitude);
    printf("\r\n");
    printf("无线接收状态: ");
    switch(YK_NRF_Rx_State) {
        case RX_DATA_OK:
            printf("接收到数据（不代表数据准确）\r\n");
            break;
        case NOT_RX_DATA:
            printf("未接收到数据\r\n");
            break;
        case RX_DATA_TRUE:
            printf("接收到数据且数据正确\r\n");
            break;
        default:
            printf("未知状态 (0x%02X)\r\n", NRF_Receive_Data.Body_Rx_State);
    }
    printf("电池电压: %.2fV\r\n", Body_Power_V);
    printf("===========================\r\n\r\n");
    
    // 同时发送格式化数据给上位机（机器可读格式）
//    UART_SendToUpperComputer();
}

void Uasrt_Show()
{
//		UART_Print_Show_State();//调试信息打印
		UART_SendToUpperComputer();//发送格式化数据给上位机（机器可读格式）
		
}


/////////////////////////////////////////////////////////////////////////OLED**************************************************************************************


//void Oled_Show()
//{
//u8g2_ClearBuffer(&u8g2);
//draw_Four_X(&u8g2,5);
//u8g2_SendBuffer(&u8g2);
//	
//}
void Oled_Show()
{
	char oled_Show_Data[30];
	u8g2_ClearBuffer(&u8g2);
	
	
	u8g2_SetFont(&u8g2, u8g2_font_unifont_t_0_75);
	sprintf(oled_Show_Data,"P%6.1f",Body_Euler_Angle.pitch );
	u8g2_DrawStr(&u8g2,0,40,oled_Show_Data);
	sprintf(oled_Show_Data,"R%6.1f", Body_Euler_Angle.roll );
	u8g2_DrawStr(&u8g2,0,51,oled_Show_Data);
	sprintf(oled_Show_Data,"Y%6.1f",Body_Euler_Angle.yaw);
	u8g2_DrawStr(&u8g2,0,63,oled_Show_Data);
	
	sprintf(oled_Show_Data,"V:%4d",L_QH_CH );
	u8g2_DrawStr(&u8g2,0,25,oled_Show_Data);
	switch(page)
	{
		case 0:
//            printf("%d,,");
			if(YK_NRF_Rx_State == RX_DATA_TRUE)
			{
				sprintf(oled_Show_Data,"YK_ROK");
				u8g2_DrawStr(&u8g2,70,10,oled_Show_Data);
				
			}
			if(NRF_Receive_Data.Body_Rx_State == RX_DATA_TRUE)
			{
				sprintf(oled_Show_Data,"Body_ROK");
				u8g2_DrawStr(&u8g2,60,30,oled_Show_Data);
				
			}

			//摇杆进度条显示
			sprintf(oled_Show_Data,"S");
			u8g2_DrawStr(&u8g2,0,10,oled_Show_Data);
			u8g2_DrawBox(&u8g2,10,0,40*L_QH_CH/4096,10);
			
		break;
		case 1:
			sprintf(oled_Show_Data,"P_a:%4.0f",Body_Gyro.gy);
			u8g2_DrawStr(&u8g2,60,10,oled_Show_Data);
			sprintf(oled_Show_Data,"R_a:%4.0f",Body_Gyro.gx);
			u8g2_DrawStr(&u8g2,60,21,oled_Show_Data);
			sprintf(oled_Show_Data,"Y_a:%4.0f",Body_Gyro.gz);
			u8g2_DrawStr(&u8g2,60,32,oled_Show_Data);

		break;
		case 2:
			//欧拉角指令显示(数字)
//			sprintf(oled_Show_Data,"P_C:%5.1f",NRF_Send_Data.control_commend.pitch);
//			u8g2_DrawStr(&u8g2,50,10,oled_Show_Data);
//			sprintf(oled_Show_Data,"R_C:%5.1f",NRF_Send_Data.control_commend.roll);
//			u8g2_DrawStr(&u8g2,50,21,oled_Show_Data);
//			sprintf(oled_Show_Data,"Y_C:%5d",NRF_Send_Data.control_commend.yaw_w);
//			u8g2_DrawStr(&u8g2,50,32,oled_Show_Data);
			//欧拉角指令显示(进度条)
			sprintf(oled_Show_Data,"P_C");
			u8g2_DrawStr(&u8g2,60,10,oled_Show_Data);
		    u8g2_DrawBox(&u8g2,90,0,20+(20.0*R_QH_CH / 2048),10);
			sprintf(oled_Show_Data,"R_C");
			u8g2_DrawStr(&u8g2,60,21,oled_Show_Data);
			u8g2_DrawBox(&u8g2,90,11,20+(20.0*R_ZY_CH / 2048),10);
			sprintf(oled_Show_Data,"Y_C");
			u8g2_DrawStr(&u8g2,60,32,oled_Show_Data);
			u8g2_DrawBox(&u8g2,90,22,20+(20.0*L_ZY_CH/ 2048),10);
		
			//油门摇杆进度条显示
			sprintf(oled_Show_Data,"S");
			u8g2_DrawStr(&u8g2,0,10,oled_Show_Data);
			u8g2_DrawBox(&u8g2,10,0,40*L_QH_CH/4096,10);
			
		break;
		default:
			break;

	}
//	
//	sprintf(oled_Show_Data,"S%d",NRF_Send_Data.Speed_Basic);
//	u8g2_DrawStr(&u8g2,0,10,oled_Show_Data);
	if(NRF_Send_Data.All_Stop ==true)
	{
		sprintf(oled_Show_Data,"STOP" );
		u8g2_DrawStr(&u8g2,90,62,oled_Show_Data);
	}
	else if(NRF_Send_Data.All_Stop == false)
	{
		sprintf(oled_Show_Data,"RUN" );
		u8g2_DrawStr(&u8g2,90,62,oled_Show_Data);
	}
	//电池电压显示
	sprintf(oled_Show_Data,"V:%3.2fV",Body_Power_V );
	u8g2_DrawStr(&u8g2,60,52,oled_Show_Data);
	
	
	u8g2_SendBuffer(&u8g2);
}
/////////////////////////////////////////////////////////////////////////按键摇杆**************************************************************************************
void Key_Task()//按键任务
{
	uint8_t Key_Down,Key_Long,Key_LongLong,Key_Up;
	Key_Scan();

	Key_Down = Get_Key_Val(Down_Mode);
	Key_Up   = Get_Key_Val(Up_Mode);
	Key_Long = Get_Key_Val(Long_Mode);
	Key_LongLong = Get_Key_Val(LongLong_Mode);
	if(Key_Down)BEEP_Sound_Number = 1;
	if(Key_Long)BEEP_Sound_Number = 2;
	if(Key_LongLong)BEEP_Sound_Number = 3;	
	

	switch(Key_Down)
	{
		case (KEY_Side_R):
			NRF_Send_Data.All_Stop = !NRF_Send_Data.All_Stop;
		break;
		case (KEY_Side_L):
			page = (page+1)%3;
		break;
//		case(KEY_Front_U):
//			switch(PID_Choose)
//			{
//				case 0:
//					PID_ID_Temp = (PID_ID_Temp+1)%4;
//				break;
//				case 1:
//					PID_Data_Temp[0]+=PID_Data_Change_Inc;
//				break;
//				case 2:
//					PID_Data_Temp[1]+=PID_Data_Change_Inc;
//				break;
//				case 3:
//					PID_Data_Temp[2]+=PID_Data_Change_Inc;
//				break;
//				
//			}
//		break;
//		case(KEY_Front_D):
//			switch(PID_Choose)
//			{
//				case 0:
//					// PID_ID_Temp = （PID_ID_Temp）%3+1
//				case 1:
//					PID_Data_Temp[0]-=PID_Data_Change_Inc;
//				break;
//				case 2:
//					PID_Data_Temp[1]-=PID_Data_Change_Inc;
//				break;
//				case 3:
//					PID_Data_Temp[2]-=PID_Data_Change_Inc;
//				break;
//				
//			}
//		break;
//		case(KEY_Front_R):
//			PID_Choose = (PID_Choose+1)%4;
//		break;
//		case(KEY_Front_L):
//			NRF_Send_Data.PID_Data[0] = PID_Data_Temp[0];
//			NRF_Send_Data.PID_Data[1] = PID_Data_Temp[1];
//			NRF_Send_Data.PID_Data[2] = PID_Data_Temp[2];
//			NRF_Send_Data.PID_ID      = PID_ID_Temp;
//		break;



	}
	
}
void Rocker_Task()//摇杆任务
{

	Rocker_Key_Scan();
	if(Get_Rocker_Key_Val(Rocker_Down_Mode,R_ZY)== Rocker_Key_Zuo){LED1_TOGGLE;}
	else if(Get_Rocker_Key_Val(Rocker_Down_Mode,R_ZY)== Rocker_Key_You){LED2_TOGGLE;}
	if(Get_Rocker_Key_Val(Rocker_Down_Mode,R_QH)== Rocker_Key_Qian){LED1_TOGGLE;}
	else if(Get_Rocker_Key_Val(Rocker_Down_Mode,R_QH)== Rocker_Key_Hou){LED2_TOGGLE;}
	if(Get_Rocker_Key_Val(Rocker_Down_Mode,L_ZY)== Rocker_Key_Zuo){LED1_TOGGLE;}
	else if(Get_Rocker_Key_Val(Rocker_Down_Mode,L_ZY)== Rocker_Key_You){LED2_TOGGLE;}
	if(Get_Rocker_Key_Val(Rocker_Down_Mode,L_QH)== Rocker_Key_Qian){LED1_TOGGLE;}
	else if(Get_Rocker_Key_Val(Rocker_Down_Mode,L_QH)== Rocker_Key_Hou){LED2_TOGGLE;}
	
	Get_Rocker_FlyCH_with_death(&L_QH_CH , &L_ZY_CH , &R_QH_CH , &R_ZY_CH);
//	NRF_Send_Data.control_commend.Speed_Basic = (0.8*Motor_PWM_Duty_Max*L_QH_CH)/4096;//油门控制
//	NRF_Send_Data.control_commend.pitch = (Pitch_Commend_Max*R_QH_CH)/2048.0f;//pitch控制
//	NRF_Send_Data.control_commend.roll = (Roll_Commend_Max*R_ZY_CH)/2048.0f;
//	NRF_Send_Data.control_commend.yaw_w = (Yaw_Commend_Max*L_ZY_CH)/2048.0f;
	NRF_Send_Data.YK_CH.CH_L_QH = L_QH_CH;
	NRF_Send_Data.YK_CH.CH_L_ZY = L_ZY_CH;
	NRF_Send_Data.YK_CH.CH_R_QH = R_QH_CH;
	NRF_Send_Data.YK_CH.CH_R_ZY = R_ZY_CH;
}
/////////////////////////////////////////////////////////////////////////蜂鸣器**************************************************************************************
void Beep_Task()//蜂鸣器任务
{
	static u8 Beep_Count = 0;
	static u8 Beep_Sound_Time = 4; 
	if(Key_BEEP_ENABLE)
	{
		if(BEEP_Sound_Number)
		{
			BEEP_ON;
			Beep_Count++;
//			printf("%d  ",Beep_Count);
			if(Beep_Count>=Beep_Sound_Time)
			{
				BEEP_OFF;
				Beep_Count = 0;
				BEEP_Sound_Number --;
			}
			
		}
	}
}
/////////////////////////////////////////////////////////////////////////PID调参**************************************************************************************
//串口直接改pid     格式：  #0,0,0,ID:1@     对应Kp,Ki,Kd,ID
void Body_PID_Change_Usart(uint8_t * data ,uint8_t Num)
{
	int ID;
	float Kp,Ki,Kd;
	if(Num<12 || data[0]!='#')
	{
		printf("err");
		return ;
	}
	sscanf((char*)data,"#%f,%f,%f,ID:%d@",&Kp,&Ki,&Kd,&ID);
	
	printf("%f,%f,%f,ID:%d\r\n",Kp,Ki,Kd,ID);
	NRF_Send_Data.PID_Data[0] = Kp;
	NRF_Send_Data.PID_Data[1] = Ki;
	NRF_Send_Data.PID_Data[2] = Kd;
	NRF_Send_Data.PID_ID =ID;

}
//////////////////////////////////////////////////////////////////////////初始化****************************************************
void ALL_Init()
{
	  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1,USART_Buff,28);
	 LED1_OFF;
  LED2_OFF;
  BEEP_OFF;
	

  NRF24L01_Init(MODEL_TX2,NRF_CH);;//无线模块
	Rocker_Init();
	u8g2Init(&u8g2);
	printf("u8g2初始化成功\r\n");
	NRF_Send_Data.All_Stop =true;
	
}
