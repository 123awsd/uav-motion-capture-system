#include "Send_Receive_Data.h"


#define BODY_RX_DATA_OK        0x01   //接收到数据（不代表数据准确）
#define BODY_NOT_RX_DATA       0x02   //未接收到数据
#define BODY_RX_DATA_TRUE      0x03  //接收到数据并且数据正确

u8  Send_JiaoYan_Hand = 0x33;
u8  Send_JiaoYan_Tail = 0x44;
u8  Receive_JiaoYan_Hand = 0x11;
u8  Receive_JiaoYan_Tail = 0x22;



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
//	memcpy(NRF_Send_Data.JiaoYan_Hand,Send_JiaoYan_Hand,2);//发送帧头
//	memcpy(NRF_Send_Data.JiaoYan_Tail,Send_JiaoYan_Tail,2);//发送帧尾
	NRF_Send_Data.Body_Rx_State = NRF_Rx_State; //发送机体接收状态
	NRF_Send_Data.Eular_Angle.pitch =(short) (Body_Euler_Angle.pitch*80);//装载欧拉角扩大80倍节约空间
	NRF_Send_Data.Eular_Angle.roll =(short) (Body_Euler_Angle.roll*80);//装载欧拉角扩大80倍节约空间
	NRF_Send_Data.Eular_Angle.yaw =(short) (Body_Euler_Angle.yaw*80);//装载欧拉角扩大80倍节约空间
	
	NRF_Send_Data.Gyro.gx = (short)(Body_Gyro.gx*80);//装载角速度扩大80倍节约空间
	NRF_Send_Data.Gyro.gy = (short)(Body_Gyro.gy*80);//装载角速度扩大80倍节约空间
	NRF_Send_Data.Gyro.gz = (short)(Body_Gyro.gz*80);//装载角速度扩大80倍节约空间
	NRF_Send_Data.Altitude = (short)(bmp280_Read_RelativeAltitude()*100);//高度扩大
	NRF_Send_Data.Power_V = (u8)(Power_Voltage*50);//电压扩大50倍
//	printf("%f\r\n",NRF_Send_Data.Eular_Angle.yaw);
	Temp.Send_Data = NRF_Send_Data;
	memcpy(NRF_Tx_Buff,Temp.Byte,32);
//		for(int i=0;i<32;i++)
//		{
//			printf("%x ", NRF_Tx_Buff[i]);
//		}
//		printf("\r\n\r\n");
}

void Body_Control_Inf_Update()//机体控制信息更新
{
	YK_CH_STR   CH_Val;
	CH_Val = NRF_Receive_Data.YK_CH;
	
	Body_Control_Inf.Attitude_Commend.Pitch = -CH_Val.CH_R_QH*Pitch_Commend_Max/2048;//pitch   //加负号调整操作杆方向
	Body_Control_Inf.Attitude_Commend.Roll = -CH_Val.CH_R_ZY*Roll_Commend_Max/2048;
	Body_Control_Inf.Attitude_Commend.Yaw_w = CH_Val.CH_L_ZY*Yaw_W_Commend_Max/2048;
	Body_Control_Inf.Attitude_Commend.Speed_Basic = CH_Val.CH_L_QH*Motor_PWM_Duty_Max/4096;
	
}
void 	NRF_Rx_Data_JX(u8 *data)
{
	typedef union
	{
		NRF_ALL_Receive_DATA    Receive_Data;
		u8 Byte[32];
	}Rx_Inf;
	Rx_Inf  RxTemp;
	
//    for(int i=0;i<32;i++)
//    {
//        printf("%x ",data[i]);
//        
//    }
//    printf("\r\n");
	memcpy(RxTemp.Byte,data,32);
	if( RxTemp.Receive_Data.JiaoYan_Hand == Receive_JiaoYan_Hand  &&
	    RxTemp.Receive_Data.JiaoYan_Tail == Receive_JiaoYan_Tail  )//帧头帧尾检测
	{
		NRF_Rx_State  = BODY_RX_DATA_TRUE;//接收到数据并且数据正确
		
		
		NRF_Receive_Data = RxTemp.Receive_Data;
		Body_Control_Inf_Update();//机体控制信息更新
	}
	
}



void Data_T_AND_R()
{
 static u16 RX_Time_Cnt = 0;
	
      if(RX_Time_Cnt <30)
    {
        NRF_Rx_State = BODY_RX_DATA_OK;
    }
    else
    {
        NRF_Rx_State = BODY_NOT_RX_DATA;
    }
    if(MY_NRF_Check_RX(NRF_Rx_Buff ) == 1)
    {
        RX_Time_Cnt = 0;
//        printf("FFF");
        NRF_Rx_Data_JX(NRF_Rx_Buff);//数据解析
        
//        NRF_Tx_Buff[0] = 0xaa;
        
//        printf("%x ",NRF_Rx_Buff[0]);
    }
    else 
    {
        RX_Time_Cnt++;
    }
    
  
    NRF_Tx_Set_Buff();//装载发送的数据
	MY_NRF240l_TxPacket_AP(NRF_Tx_Buff,32);//利用应答发送	
		
//	printf("%d  ",NRF_Rx_State);
}
	



