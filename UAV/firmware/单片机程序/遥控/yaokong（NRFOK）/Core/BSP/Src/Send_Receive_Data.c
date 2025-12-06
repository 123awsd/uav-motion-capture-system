#include "Send_Receive_Data.h"





u8  Send_JiaoYan_Hand = 0x11;
u8  Send_JiaoYan_Tail = 0x22;
u8  Receive_JiaoYan_Hand = 0x33;
u8  Receive_JiaoYan_Tail = 0x44;

void NRF_Send_Data_Init()
{
	NRF_Send_Data.All_Stop = true;
	NRF_Send_Data.JiaoYan_Hand = Send_JiaoYan_Hand;
	NRF_Send_Data.JiaoYan_Tail = Send_JiaoYan_Tail;
	
}

//将数据装入发送数组
void NRF_Tx_Set_Buff()
{
	typedef union
	{
		NRF_ALL_SEND_DATA    Send_Data;
		u8 Byte[32];
	}Tx_Inf;
	Tx_Inf  Temp;
	
	NRF_Send_Data.JiaoYan_Hand = Send_JiaoYan_Hand;//发送帧头
	NRF_Send_Data.JiaoYan_Tail = Send_JiaoYan_Tail;//发送帧尾
//	memcpy(NRF_Send_Data.JiaoYan_Hand,Send_JiaoYan_Hand,2);
//	memcpy(NRF_Send_Data.JiaoYan_Tail,Send_JiaoYan_Tail,2);
	Temp.Send_Data = NRF_Send_Data;
	memcpy(NRF_Tx_Buff,Temp.Byte,32);
    
    
}
	
void 	NRF_Rx_Data_JX(u8 *data)
{
	typedef union
	{
		NRF_ALL_Receive_DATA    Receive_Data;
		u8 Byte[32];
	}Rx_Inf;
	Rx_Inf  RxTemp;
	memset(RxTemp.Byte,3,32);
	memcpy(RxTemp.Byte,data,32);
//					for(int i=0;i<32;i++)
//		{
//			printf("%x ", RxTemp.Byte[i]);
//		}
//		printf("\r\n");
//printf("aaa");	
//	printf("A       %x   %x",  RxTemp.Receive_Data.JiaoYan_Hand ,RxTemp.Receive_Data.JiaoYan_Tail);//帧头帧尾检测
	if( RxTemp.Receive_Data.JiaoYan_Hand == Receive_JiaoYan_Hand  &&
	    RxTemp.Receive_Data.JiaoYan_Tail == Receive_JiaoYan_Tail   )//帧头帧尾检测
        {
		YK_NRF_Rx_State  = RX_DATA_TRUE;//接收到数据并且数据正确
		NRF_Receive_Data = RxTemp.Receive_Data;
			
		//****数据修正	
		Body_Power_V = (float)(NRF_Receive_Data.Power_V /50.f);
		Body_Altitude = 	NRF_Receive_Data.Altitude/100.f;
		 //修正角度
//			printf("\r\nAAAAAA:%d\r\n",NRF_Receive_Data.Eular_Angle.yaw);
		Body_Euler_Angle.yaw = (float)NRF_Receive_Data.Eular_Angle.yaw / 80.0f;
		Body_Euler_Angle.pitch = (float)NRF_Receive_Data.Eular_Angle.pitch / 80.0f;
		Body_Euler_Angle.roll = (float)NRF_Receive_Data.Eular_Angle.roll / 80.0f;
		// 修正角速度
		Body_Gyro.gx = (float)NRF_Receive_Data.Gyro.gx / 80.0f;
		Body_Gyro.gy = (float)NRF_Receive_Data.Gyro.gy / 80.0f;
		Body_Gyro.gz = (float)NRF_Receive_Data.Gyro.gz / 80.0f;	
//			printf("航向角(yaw): %f\r\n", NRF_Receive_Data.Eular_Angle.yaw);
//		printf("%d,%d,%d,%d ,%d %d %d",RxTemp.Byte[2],RxTemp.Byte[3],RxTemp.Byte[4],RxTemp.Byte[5],RxTemp.Byte[6],RxTemp.Byte[7],RxTemp.Byte[8]);
//				for(int i=0;i<32;i++)
//		{
//			printf("%x ", RxTemp.Byte[i]);
//		}

		
//		printf("  %d  ",RxTemp.Receive_Data.Body_Rx_State);
//	    printf("  %d",  RxTemp.Receive_Data.JiaoYan_Tail[0] );
//		printf("%lf\r\n",NRF_Receive_Data.Eular_Angle.pitch);
//		printf("%f\r\n",a);
//		printf("  %d",  RxTemp.Receive_Data.JiaoYan_Tail[1] );
//		printf("\r\n");
	}
	
}

void Data_Send_To_Body()
{
    NRF_Tx_Set_Buff();//装载发送的数据
    MY_NRF_TxPacket(NRF_Tx_Buff,32);//发送数据
}


void Data_Receive_Form_Body()
{
     static u16 RX_Time_Cnt = 0;
    
    
     if(RX_Time_Cnt <20 )
    {
//         YK_NRF_Rx_State = RX_DATA_OK;
    }
    else
    {
        YK_NRF_Rx_State = NOT_RX_DATA;//未接收到数据
    }
    if(MY_NRF_Check_RX(NRF_Rx_Buff) == 1)
    {
        
//        printf("FFF");
        RX_Time_Cnt = 0;
        NRF_Rx_Data_JX(NRF_Rx_Buff);//数据解析
    }
    else
    {
        RX_Time_Cnt ++;    
    }
// printf("%d  \r\n",RX_Time_Cnt);
//    printf("%d  \r\n",YK_NRF_Rx_State);
}

//切换模式接收方式，不好用
//void Data_T_AND_R()
//{
//	static u8 R_T_Flag = 1,NRF_Count = 0; //接收发送标志 1为接收 0 为发送
//	u8 NRF_state;
//	
//	if(R_T_Flag == TX_Mode )
//	{
//		NRF_state = NRF24L01_TxPacket(NRF_Tx_Buff);
//		if(NRF_state == TX_OK || NRF_state == MAX_TX)
//		{
//			NRF24L01_RX_Mode();//切换为接收模式
//			R_T_Flag = RX_Mode;
//			
//		}
//	}
//	else if(R_T_Flag == RX_Mode)
//	{
//		NRF_state = NRF24L01_RxPacket(NRF_Rx_Buff);
//		NRF_Count++;
//		
//		if(NRF_state == RX_OK || NRF_Count >=40)
//		{
////			if(NRF_state == 0)printf("%d\r\n",Count++);
//			if(NRF_Count >=40)YK_NRF_Rx_State = NOT_RX_DATA;
//			if(NRF_state == RX_OK) 
//			{
//				YK_NRF_Rx_State = RX_DATA_OK;
//				NRF_Rx_Data_JX(NRF_Rx_Buff);//数据解析
//			}
//				
//			NRF24L01_TX_Mode();//切换为发送模式
//			NRF_Tx_Set_Buff();//装载发送的数据
//			R_T_Flag = TX_Mode;
//			NRF_Count = 0;
//		}
//	}
//	
//}
	



