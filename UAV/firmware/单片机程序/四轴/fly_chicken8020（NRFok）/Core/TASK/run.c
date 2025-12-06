#include "run.h"
//NRF接收发送的数组

void ALL_Init()
{

   //***************************外设初始化	 
		bmp280_Init();//气压计初始化
   My_Mpu_Init();//mpu初始化
   NRF24L01_Init(MODEL_RX2,NRF_CH);//主接收

	HAL_Delay(30);//等待气压计初始化后设置相对高度
	bmp280_SetRelativeZero();
   //*******************************pid初始化
	Body_Balance_PID_Init();
	NRF_Receive_Data.All_Stop = true;

}







void Drone_Control_Task()//电机控制
{
	ATTITUDE_CONTROL_STR   Attitude_Con_Cmd;//姿态控制指令
	if(NRF_Receive_Data.All_Stop == true)//急停
	{
		Motor_Duty.m1 = 0;
		Motor_Duty.m2 = 0;
		Motor_Duty.m3 = 0;
		Motor_Duty.m4 = 0;
		Body_Balance_PID_Isum_Clear();//积分清零
		Motor_Set_Duty(Motor_Duty.m1 , Motor_Duty.m2 , Motor_Duty.m3 , Motor_Duty.m4);
		return ;
	}
	else
	{
		//*********************************自稳模式*******************************************************************************************************************************//
		Attitude_Con_Cmd = Body_Control_Inf.Attitude_Commend;
		Mode_Attitude_600Hz(Attitude_Con_Cmd.Roll,Attitude_Con_Cmd.Pitch,Attitude_Con_Cmd.Yaw_w,Attitude_Con_Cmd.Speed_Basic);
		//*********************************无控制电机测试******************************************************************************//
//		Attitude_Con_Cmd = Body_Control_Inf.Attitude_Commend;
//		Motor_Duty.m1 = Attitude_Con_Cmd.Speed_Basic*1000/4096;
//		Motor_Duty.m2 = Attitude_Con_Cmd.Speed_Basic*1000/4096;
//		Motor_Duty.m3 = Attitude_Con_Cmd.Speed_Basic*1000/4096;
//		Motor_Duty.m4 = Attitude_Con_Cmd.Speed_Basic*1000/4096;
	}
}


//******************************************************************************************PID参数调整********************************************************
// PID参数调整1:角速度环，，2：角度环
static	float roll_rate_exp  = 1.f; // 遥控量转角速度期望
	float pitch_rate_exp = 0.f;
	float yaw_rate_exp   = 0.f;

	float roll_exp  = 0.f; // 遥控量转角度期望
	float pitch_exp = 0.f;
	float yaw_exp   = 0.f;  // 偏航角速度期望
void Drone_PID_Parameter_Tuning_600HZ(u8 PID_Mode)
{
		ATTITUDE_CONTROL_STR   Attitude_Con_Cmd;//姿态控制指令
		Attitude_Con_Cmd = Body_Control_Inf.Attitude_Commend;
    // 急停判断
    if (NRF_Receive_Data.All_Stop == true)
    {
        Motor_Duty.m1 = Motor_Duty.m2 = Motor_Duty.m3 = Motor_Duty.m4 = 0;
        Body_Balance_PID_Isum_Clear();     // 积分清零
        Motor_Set_Duty(0, 0, 0, 0);       // 电机停转
        return;
    }
//		else
//		{
//		        Motor_Set_Duty(0, 0, 1000, 0);       // 电机停转
//        return;
//		}
    // 基础油门
    uint16_t basic_speed = 2200;
    // 更新陀螺仪数据
    Imu_Get_Gyro_Data(&Body_Gyro.gx, &Body_Gyro.gy, &Body_Gyro.gz);

    switch (PID_Mode)
    {
        case 1: // 调内环：角速度环
        {
						int16_t m1, m2, m3, m4;
					
					  roll_rate_exp = Attitude_Con_Cmd.Roll*500/30;//****************************选择对应的轴
            Body_Rate_Control(roll_rate_exp, pitch_rate_exp, yaw_rate_exp,basic_speed, &m1, &m2, &m3, &m4);
            Motor_Set_Duty(m1, m2, m3, m4);
        }
        break;

        case 2: // 调外环：角度模式（外环200Hz+内环600Hz）
        {
						roll_exp = Body_Control_Inf.Attitude_Commend.Roll;
            Mode_Attitude_600Hz(roll_exp, pitch_exp, yaw_exp, basic_speed);
        }
        break;

        default:
            Motor_Set_Duty(0, 0, 0, 0);
        break;
    }
}

void Drone_PID_Parameter_Usart_show()
{
	//角速度环打印
//	printf("%f,%f\r\n",roll_rate_exp,Body_Gyro.gx);//ROLL
//	printf("%f,%f\r\n",pitch_rate_exp,Body_Gyro.gy);//PITCH
//	printf("%f,%f\r\n",yaw_rate_exp,Body_Gyro.gz);//YAW
	//角度环打印
	printf("%f,%f\r\n",roll_exp,Body_Euler_Angle.roll);//ROLL
//	printf("%f,%f\r\n",pitch_exp,Body_Euler_Angle.pitch);//PITCH
		
}
void Eular_Angle_Update(void)//欧拉角数据更新
{
	Imu_Euler_Angle_Update(0);//欧拉角解析
	Imu_Get_Euler_Angle_Data(&Body_Euler_Angle.pitch , &Body_Euler_Angle.roll , &Body_Euler_Angle.yaw);
	Imu_Get_Gyro_Data(&Body_Gyro.gx , &Body_Gyro.gy , &Body_Gyro.gz);
}

//获取机体电压
float Get_Body_Power()
{
    uint16_t adc_data;
    float voltage;
    
    // 启动ADC并获取原始值
    HAL_ADC_Start(&hadc1);
    adc_data = HAL_ADC_GetValue(&hadc1);
    
    // ADC转换公式：电压 = (ADC值 / ADC最大量程) * 参考电压 * 分压系数
    // 假设：12位ADC（最大4095），参考电压3.3V，若有分压电路需乘以分压系数
    // 例如：若电池电压经过1:2分压（如两个相同电阻串联），分压系数为2
//    voltage = (adc_data * 3.3f) / 4095.0f;  // 基础转换（无分压时）
    voltage = (adc_data * 3.3f * 2) / 4095.0f;  // 有1:2分压时使用
    
    return voltage;
}

void Uasrt_Show()
{
//	flow_show();
//	printf("%f,%f,%f\r\n",Body_Euler_Angle.pitch ,  Body_Euler_Angle.roll ,Body_Euler_Angle.yaw  );
//	printf("%d\r\n",NRF_Rx_State);
//printf("%f,%f,%f,%d\r\n",NRF_Receive_Data.PID_Data[0] , NRF_Receive_Data.PID_Data[1] , NRF_Receive_Data.PID_Data[2] ,NRF_Receive_Data.PID_ID);
//	printf("%f,%f,%f\r\n",Body_Gyro.gx,Body_Gyro.gy,Body_Gyro.gz);
//	Body_Balance_Col_Usart_Show();
//	printf("%.2f\r\n",Power_Voltage);
//	printf("%f,%f,",Body_Control_Inf.Attitude_Commend.Roll , Body_Control_Inf.Attitude_Commend.Yaw_w );
//	printf("%f,%d\r\n",Body_Control_Inf.Attitude_Commend.Pitch ,Body_Control_Inf.Attitude_Commend.Speed_Basic );
//	printf("%d\r\n",NRF_Receive_Data.YK_CH.CH_L_QH);
	
	
	    //************************** 打印收数据信息**********************//
//		printf("===== NRF接收数据信息 =====\r\n");
//    
//    // 打印校验信息
//    printf("帧头: 0x%02X\r\n", NRF_Receive_Data.JiaoYan_Hand);
//    printf("帧尾: 0x%02X\r\n", NRF_Receive_Data.JiaoYan_Tail);
//    printf("\r\n");
//    
//    // 打印遥控通道值（使用直接访问方式）
//    printf("【遥控原始通道值】\r\n");
//    printf("右摇杆前后舵量: %d\r\n", NRF_Receive_Data.YK_CH.CH_R_QH);
//    printf("右摇杆左右舵量: %d\r\n", NRF_Receive_Data.YK_CH.CH_R_ZY);
//    printf("左摇杆前后舵量: %d\r\n", NRF_Receive_Data.YK_CH.CH_L_QH);
//    printf("左摇杆左右舵量: %d\r\n", NRF_Receive_Data.YK_CH.CH_L_ZY);
//    printf("\r\n");
//    
//    // 打印急停状态
//    printf("急停状态: %s\r\n", NRF_Receive_Data.All_Stop ? "已触发" : "正常");
//    printf("\r\n");
//    
//    // 打印PID参数（直接访问数组元素）
//    printf("【PID参数】\r\n");
//    printf("调试ID: %d\r\n", NRF_Receive_Data.PID_ID);
//    printf("Kp: %.2f\r\n", NRF_Receive_Data.PID_Data[0]);
//    printf("Ki: %.2f\r\n", NRF_Receive_Data.PID_Data[1]);
//    printf("Kd: %.2f\r\n", NRF_Receive_Data.PID_Data[2]);
//    printf("===========================\r\n\r\n");
//printf("P:%.2f\r\n",bmp280_Read_RelativeAltitude());
	
//	printf("Power:%.2f\r\n",Get_Body_Power());
//	printf("yuan: %.2f\r\n", Body_Euler_Angle.yaw);
//	printf("fa: %d\r\n", NRF_Send_Data.Eular_Angle.yaw);
	
	
	
		    //************************** PID调整**********************//
	
	
	
	
	
	
	
	
	
	
}

void PID_Change(void)//pid调参
{
	Body_Balance_PID_Change_NRF();

}



