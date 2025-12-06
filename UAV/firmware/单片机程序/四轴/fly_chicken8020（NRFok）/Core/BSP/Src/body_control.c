#include "body_control.h"

#define Speed_I_Div    1000 //积分分离，遥控器油门低于这个值就不进行积分运算，积分清零



//姿态控制
PID_Str  Pos_X_Control_PID , Pos_Y_Control_PID,Pos_Z_Control_PID;//位置环
PID_Str  Speed_X_Control_PID , Speed_Y_Control_PID,Speed_Z_Control_PID;//速度环
PID_Str  Yaw_Control_PID , Roll_Control_PID,Pitch_Control_PID;//角度外环
PID_Str  Gyro_Yaw_Control_PID , Gyro_Roll_Control_PID , Gyro_Pitch_Control_PID;//角速度内环
int16_t M1,M2,M3,M4;
float Yaw_Out=0,Pitch_Out=0,Roll_Out=0;




void Body_Balance_PID_Init()
{
//外环角度
    PID_Init(3,0,0,300,-300,300,&Yaw_Control_PID);
    PID_Init(3.7,0,0.1,3000,-500,500,&Roll_Control_PID);   // #0.9,0.001,0.18,ID:2@ #2.9,0,-0.1,ID:2@       1600油门：#1.5,0,0,ID:2@   #0.7,0.001,0,ID:2@
    PID_Init(3.7,0,0.1,3000,-500,500,&Pitch_Control_PID);//#0.4,0.0009,-0.0003,ID:3@          #1.9,0,0,ID:3@     1600油门：#1,0,0,ID:3@

//	  PID_Init(0,0,0,3000,-500,500,&Roll_Control_PID);   // #0.9,0.001,0.18,ID:2@ #2.9,0,-0.1,ID:2@       1600油门：#1.5,0,0,ID:2@   #0.7,0.001,0,ID:2@
//    PID_Init(0,0,0,3000,-500,500,&Pitch_Control_PID);//#0.4,0.0009,-0.0003,ID:3@          #1.9,0,0,ID:3@     1600油门：#1,0,0,ID:3@
	
	
//内环角速度
	  PID_Init(24,0,0,3000,-1000,1000,&Gyro_Yaw_Control_PID);
    PID_Init(24,0.05,4,4000,-5000,5000,&Gyro_Roll_Control_PID);    //  -9 0.2     1600油门：          #-5,-0.15,0,ID:5@
    PID_Init(24,0.05,4,4000,-5000,5000,&Gyro_Pitch_Control_PID);//#12,0.25,10,ID:6@  1600油门：6.3,0.14,0
	
//			PID_Init(0,0,0,3000,-1000,1000,&Gyro_Yaw_Control_PID);
//	  PID_Init(0,0,0,4000,-5000,5000,&Gyro_Roll_Control_PID);    //  -9 0.2     1600油门：          #-5,-0.15,0,ID:5@
//    PID_Init(0,0,0,4000,-5000,5000,&Gyro_Pitch_Control_PID);//#12,0.25,10,ID:6@  1600油门：6.3,0.14,0

}

void Body_Balance_PID_Isum_Clear(void)
{
	Gyro_Pitch_Control_PID.Isum = 0;
	Gyro_Roll_Control_PID.Isum = 0;
	Gyro_Yaw_Control_PID.Isum = 0;
	Yaw_Control_PID.Isum = 0;
	Roll_Control_PID.Isum = 0;
	Pitch_Control_PID.Isum = 0;
}


//=============************************************************************************=== 位置环控制 ====******************************************************************************================
/**
 * @brief  位置环（位置 → 姿态角 / 油门）
 * @param  X_EXP, Y_EXP, Z_EXP  期望位置 (m)
 * @param  Px, Py, Pz           实际位置 (m)
 * @param  Roll_EXP, Pitch_EXP  输出姿态角期望 (度)
 * @param  Basic_Speed          输入/输出油门基值 (0~4095)
 */
void Body_Position_Control(float X_EXP, float Y_EXP, float Z_EXP,
                           float Px,    float Py,    float Pz,
                           float *Roll_EXP, float *Pitch_EXP, uint16_t *Basic_Speed)
{
    Pos_X_Control_PID.Mea = Px;
    Pos_X_Control_PID.Exp = X_EXP;
    PID_Update_Pos(&Pos_X_Control_PID, 1);
    float roll_cmd = Pos_X_Control_PID.Out;   // 输出单位：度

    Pos_Y_Control_PID.Mea = Py;
    Pos_Y_Control_PID.Exp = Y_EXP;
    PID_Update_Pos(&Pos_Y_Control_PID, 1);
    float pitch_cmd = Pos_Y_Control_PID.Out;  // 输出单位：度

    Pos_Z_Control_PID.Mea = Pz;
    Pos_Z_Control_PID.Exp = Z_EXP;
    PID_Update_Pos(&Pos_Z_Control_PID, 1);
    float throttle_delta = Pos_Z_Control_PID.Out;

    *Roll_EXP  = roll_cmd;
    *Pitch_EXP = pitch_cmd;
    *Basic_Speed = (uint16_t)((int32_t)(*Basic_Speed) + (int32_t)throttle_delta);
}


//=============****************************************************************=== 角度环（外环）：生成角速度期望 ====******************************************************************************************================
/**
 * @brief  角度环：角度 → 角速度期望
 * @param  Roll_EXP, Pitch_EXP   期望角度(°)
 * @param  Yaw_Rate_EXP          期望偏航角速度(°/s)
 * @param  Basic_Speed           基础油门(0~4095)
 * @param  pRoll_Rate_EXP        输出横滚角速度期望(°/s)
 * @param  pPitch_Rate_EXP       输出俯仰角速度期望(°/s)
 * @param  pYaw_Rate_EXP         输出偏航角速度期望(°/s)
 */
void Body_Attitude_Control(float Roll_EXP, float Pitch_EXP, float Yaw_Rate_EXP, float Basic_Speed,
                           float *pRoll_Rate_EXP, float *pPitch_Rate_EXP, float *pYaw_Rate_EXP)
{
    // 低油门时清除积分，防止地面积分累积
    if (Basic_Speed < Speed_I_Div)
        Body_Balance_PID_Isum_Clear();

    // 更新姿态测量与期望
    Roll_Control_PID.Mea  = Body_Euler_Angle.roll;
    Roll_Control_PID.Exp  = Roll_EXP;
    Pitch_Control_PID.Mea = Body_Euler_Angle.pitch;
    Pitch_Control_PID.Exp = Pitch_EXP;

    // 计算角度环输出（带角速度前馈）
    PID_Update_Pos_Fly(&Pitch_Control_PID, Body_Gyro.gy);
    PID_Update_Pos_Fly(&Roll_Control_PID,  Body_Gyro.gx);

    // 输出角速度期望供内环使用
    *pRoll_Rate_EXP  = Roll_Control_PID.Out;
    *pPitch_Rate_EXP = Pitch_Control_PID.Out;
    *pYaw_Rate_EXP   = Yaw_Rate_EXP;
}





//============***********************************************************==== 角速度环（内环）：跟随角速度期望并混控到电机 ============***************************************************========
/**
 * @brief  角速度环（角速度 → 电机PWM输出），适配外环直接给三轴角速度期望
 * @param  Roll_w_EXP  期望横滚角速度 (deg/s)
 * @param  Pitch_w_EXP 期望俯仰角速度 (deg/s)
 * @param  Yaw_w_EXP   期望偏航角速度 (deg/s)
 * @param  Basic_Speed 基础油门 (0~4095)
 * @param  pM1..pM4    输出各电机PWM指针（X型：M1左前 M2右前 M3左后 M4右后）
 */
void Body_Rate_Control(float Roll_w_EXP, float Pitch_w_EXP, float Yaw_w_EXP, uint16_t Basic_Speed,
                       int16_t *pM1, int16_t *pM2, int16_t *pM3, int16_t *pM4)
{
    /* 期望与测量（单位：deg/s） */
    Gyro_Roll_Control_PID.Exp  = Roll_w_EXP;
    Gyro_Pitch_Control_PID.Exp = Pitch_w_EXP;
    Gyro_Yaw_Control_PID.Exp   = Yaw_w_EXP;

    Gyro_Roll_Control_PID.Mea  = Body_Gyro.gx;
    Gyro_Pitch_Control_PID.Mea = Body_Gyro.gy;
    Gyro_Yaw_Control_PID.Mea   = Body_Gyro.gz;

    /* 内环 PID 更新 */
    PID_Update_Pos(&Gyro_Roll_Control_PID, 1);
    PID_Update_Pos(&Gyro_Pitch_Control_PID, 1);
    PID_Update_Pos(&Gyro_Yaw_Control_PID,   1);

		//X 型混控（1右前，2左后，3左前，4右后）
		int16_t m1 = (int16_t)(Basic_Speed - Gyro_Yaw_Control_PID.Out + Gyro_Roll_Control_PID.Out + Gyro_Pitch_Control_PID.Out); // 右前
		int16_t m2 = (int16_t)(Basic_Speed - Gyro_Yaw_Control_PID.Out - Gyro_Roll_Control_PID.Out - Gyro_Pitch_Control_PID.Out); // 左后
		int16_t m3 = (int16_t)(Basic_Speed + Gyro_Yaw_Control_PID.Out - Gyro_Roll_Control_PID.Out + Gyro_Pitch_Control_PID.Out); // 左前
		int16_t m4 = (int16_t)(Basic_Speed + Gyro_Yaw_Control_PID.Out + Gyro_Roll_Control_PID.Out - Gyro_Pitch_Control_PID.Out); // 右后


    /* 限幅 */
    m1 = limitt(m1, 0, Motor_PWM_Duty_Max);
    m2 = limitt(m2, 0, Motor_PWM_Duty_Max);
    m3 = limitt(m3, 0, Motor_PWM_Duty_Max);
    m4 = limitt(m4, 0, Motor_PWM_Duty_Max);

    *pM1 = m1; *pM2 = m2; *pM3 = m3; *pM4 = m4;
}


/*==============********************************************************************************====== 无人机控制模式***********************************************************************************====================*/
/* 目标频率：位置 50 Hz，姿态 200 Hz，角速度 600 Hz（主频） */
#define POS_DECIM  12   // 50 Hz = 600/12
#define ATT_DECIM   3   // 200 Hz = 600/3
/* 角速度环 600 Hz：每个 tick 都执行 */
/*==================== 1.角度模式：角度 → 角速度 → 电机 ====================*/
void Mode_Attitude_600Hz(float Roll_EXP, float Pitch_EXP,
                         float Yaw_w_EXP, uint16_t Basic_Speed)
{
    static uint32_t tick = 0; tick++;

    static float roll_rate_exp = 0.0f, pitch_rate_exp = 0.0f, yaw_rate_exp = 0.0f;

    /* 600Hz 更新IMU角速度测量 */
		Mpu_Data_Update();
    Imu_Get_Gyro_Data(&Body_Gyro.gx, &Body_Gyro.gy, &Body_Gyro.gz);

    /* 200Hz：角度环 → 三轴角速度期望 */
    if ((tick % ATT_DECIM) == 0) {
        Body_Attitude_Control(Roll_EXP, Pitch_EXP, Yaw_w_EXP, Basic_Speed,
                              &roll_rate_exp, &pitch_rate_exp, &yaw_rate_exp);
    }

    /* 600Hz：角速度环 → 电机PWM */
    int16_t m1, m2, m3, m4;
    Body_Rate_Control(roll_rate_exp, pitch_rate_exp, yaw_rate_exp, Basic_Speed,
                      &m1, &m2, &m3, &m4);
    Motor_Set_Duty(m1, m2, m3, m4);
}

/*==================== 2.位置模式：位置 → 角度 → 角速度 → 电机 ====================*/
void Mode_Position_600Hz(float X_EXP, float Y_EXP, float Z_EXP,
                         float Px, float Py, float Pz,
                         float Yaw_w_EXP, uint16_t Basic_Speed)
{
    static uint32_t tick = 0; tick++;

    static float roll_exp = 0.0f, pitch_exp = 0.0f;
    static uint16_t base_speed = 0;

    static float roll_rate_exp = 0.0f, pitch_rate_exp = 0.0f, yaw_rate_exp = 0.0f;

    /* 600Hz 更新IMU角速度测量 */
    Imu_Get_Gyro_Data(&Body_Gyro.gx, &Body_Gyro.gy, &Body_Gyro.gz);

    /* 50Hz：位置环 → 姿态角/油门 */
    if ((tick % POS_DECIM) == 0) {
        base_speed = Basic_Speed;
        Body_Position_Control(X_EXP, Y_EXP, Z_EXP,
                              Px, Py, Pz,
                              &roll_exp, &pitch_exp, &base_speed);
    }

    /* 200Hz：角度环 → 三轴角速度期望 */
    if ((tick % ATT_DECIM) == 0) {
        Body_Attitude_Control(roll_exp, pitch_exp, Yaw_w_EXP, base_speed,
                              &roll_rate_exp, &pitch_rate_exp, &yaw_rate_exp);
    }

    /* 600Hz：角速度环 → 电机PWM */
    int16_t m1, m2, m3, m4;
    Body_Rate_Control(roll_rate_exp, pitch_rate_exp, yaw_rate_exp, base_speed,
                      &m1, &m2, &m3, &m4);
    Motor_Set_Duty(m1, m2, m3, m4);
}
//*************************************************************************************PID调参************************************************************************************//
//单独跑角速度环，用于内环参数确定
void Mode_PID_RateOnly_500Hz(float roll_rate_exp,
                         float pitch_rate_exp,
                         float yaw_rate_exp,
                         uint16_t basic_speed)
{


}



void Body_Balance_PID_Change(float Kp,float Ki,float Kd , uint8_t ID )
{

    switch(ID)
        {
            case 0:
            break;
			//外环
            case 1:
                Yaw_Control_PID.Kp = Kp;
                Yaw_Control_PID.Ki = Ki;
                Yaw_Control_PID.Kd = Kd;
            break;
            case 2:
                Roll_Control_PID.Kp = Kp;
                Roll_Control_PID.Ki = Ki;
                Roll_Control_PID.Kd = Kd;
            break;
            case 3:
                Pitch_Control_PID.Kp = Kp;
                Pitch_Control_PID.Ki = Ki;
                Pitch_Control_PID.Kd = Kd;
            break;
			//内环
			 case 4:
                Gyro_Yaw_Control_PID.Kp = Kp;
                Gyro_Yaw_Control_PID.Ki = Ki;
                Gyro_Yaw_Control_PID.Kd = Kd;
            break;
            case 5:
                Gyro_Roll_Control_PID.Kp = Kp;
                Gyro_Roll_Control_PID.Ki = Ki;
                Gyro_Roll_Control_PID.Kd = Kd;
            break;
            case 6:
                Gyro_Pitch_Control_PID.Kp = Kp;
                Gyro_Pitch_Control_PID.Ki = Ki;
                Gyro_Pitch_Control_PID.Kd = Kd;
            break;
			case 7://全部参数清零
				
//				My_Mpu_Init();
			
				Roll_Control_PID.Kp = 0;
                Roll_Control_PID.Ki = 0;
                Roll_Control_PID.Kd = 0;
				
			    Pitch_Control_PID.Kp = 0;
                Pitch_Control_PID.Ki = 0;
                Pitch_Control_PID.Kd = 0;
				
				Gyro_Yaw_Control_PID.Kp = 0;
                Gyro_Yaw_Control_PID.Ki = 0;
                Gyro_Yaw_Control_PID.Kd = 0;
			
				Gyro_Roll_Control_PID.Kp = 0;
                Gyro_Roll_Control_PID.Ki = 0;
                Gyro_Roll_Control_PID.Kd = 0;
			
				Gyro_Pitch_Control_PID.Kp = 0;
                Gyro_Pitch_Control_PID.Ki = 0;
                Gyro_Pitch_Control_PID.Kd = 0;
			
			break;
        }

}

//串口直接改pid     格式：  #0,0,0,ID:1@     对应Kp,Ki,Kd,ID
void Body_Balance_PID_Change_Usart(uint8_t * data ,uint8_t Num)
{
	int ID;
	float Kp,Ki,Kd;
	if(Num<12 || data[0]!='#')return ;
	sscanf((char*)data,"#%f,%f,%f,ID:%d@",&Kp,&Ki,&Kd,&ID);
	
	printf("%f,%f,%f,ID:%d\r\n",Kp,Ki,Kd,ID);
	Body_Balance_PID_Change(Kp,Ki,Kd,ID);
}



//遥控器改pid
void Body_Balance_PID_Change_NRF()
{
	Body_Balance_PID_Change(NRF_Receive_Data.PID_Data[0],NRF_Receive_Data.PID_Data[1],NRF_Receive_Data.PID_Data[2] , NRF_Receive_Data.PID_ID );
}









void Body_Balance_Col_Usart_Show()
{
//	static int Count =0;
//	printf("%f,%f,%f\r\n", Yaw_Control_PID.Kp ,  Yaw_Control_PID.Ki , Yaw_Control_PID.Kd);
//	printf("%f,%f,%f\r\n", Roll_Control_PID.Kp ,  Roll_Control_PID.Ki , Roll_Control_PID.Kd);
//	printf("%f,%f,%f\r\n", Pitch_Control_PID.Kp ,  Pitch_Control_PID.Ki , Pitch_Control_PID.Kd);
//	printf("%f,%f,%f,%f\r\n",Body_Euler_Angle.pitch , Pitch_Control_PID.Out,Body_Euler_Angle.roll , Roll_Control_PID.Out);
//	printf("%d,%d,%d,%d\r\n",M1,M2,M3,M4);
//	printf("%f,%f,%f\r\n",Gyro_Pitch_Control_PID.Isum,Gyro_Pitch_Control_PID.Out,Gyro_Pitch_Control_PID.Err);
//	printf("%f,%f,%f,%f,%f\r\n",Pitch_Control_PID.Out ,Pitch_Control_PID.Err, Pitch_Control_PID.Isum , Gyro_Pitch_Control_PID.Out , Gyro_Pitch_Control_PID.Isum);
//	printf("")
////	
//	printf("G:%f,%f,  %f     %d\r\n",Gyro_Roll_Control_PID.Out , Gyro_Roll_Control_PID.Isum   ,Gyro_Roll_Control_PID.Mea  ,Test_count);
//	printf("A:%f,%f,\r\n",Roll_Control_PID.Out , Gyro_Roll_Control_PID.Isum);
//	printf("\r\n");
	
//	printf("%d\r\n",Body_Control_Inf.Attitude_Commend.Speed_Basic);
}

