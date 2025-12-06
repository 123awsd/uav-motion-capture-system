#include "task.h"


TASK_INF_TYPEDEF  Task_1;
TASK_INF_TYPEDEF  Task_2;
TASK_INF_TYPEDEF  Task_3;
TASK_INF_TYPEDEF  Task_4;
TASK_INF_TYPEDEF  Task_5;
//TASK_INF_TYPEDEF  Task_6;
//TASK_INF_TYPEDEF  Task_7;
//TASK_INF_TYPEDEF  Task_8;
//TASK_INF_TYPEDEF  Task_9;
//TASK_INF_TYPEDEF  Task_10;



//若将所有任务的优先级都配置成一样，就相当于在定时器里依次按照频率调用，这样每个任务的频率都可以在（1~1000）
void MY_Task_Begin()
{

	Tsak_Init();//任务初始化
	
	Task_1 = Add_Task( 100 ,1 ,true); //创建任务1 频率，优先级，是否执行  
	Task_2 = Add_Task( 200 ,3 ,true); //创建任务2
	Task_3 = Add_Task( 500 ,2 ,true); //创建任务3 
	Task_4 = Add_Task( 1 ,5 ,true); //创建任务4 
	Task_5 = Add_Task( 50 ,6 ,true); //创建任务5 
//	Task_6 = Add_Task( 4 ,1 ,true); //创建任务6  
//	Task_7 = Add_Task( 4 ,4 ,true); //创建任务7 
//	Task_8 = Add_Task( 4 ,10 ,true); //创建任务8
//	Task_9 = Add_Task( 4 ,1 ,true); //创建任务9  
//	Task_10 = Add_Task( 4 ,1 ,true); //创建任务10 



	TASK_Run();
} 


int EXPL,EXPR;
//u8 NRF_Test[32]="asdf" ;
//u8 NRF_Tx_Buff[32]="1234";//发送数组
void Run_Task_1()
{
		Data_T_AND_R();//发送接收数据

}
void Run_Task_2()//IMU和气压计更新
{
//	printf("FFF");
		Eular_Angle_Update();
//	bmp280_GetTemperature(); 
		bmp280_GetRelativeAltitude();
}
void Run_Task_3()//控制
{
Drone_Control_Task();//控制
//	Drone_PID_Parameter_Tuning_600HZ(2);//调参

}
void Run_Task_4()//电压测量
{
	Power_Voltage = Get_Body_Power();
	
}

void Run_Task_5()//
{
		Drone_PID_Parameter_Usart_show();
//	printf("aaaa");
}
void While_Loop()//放在while(1)运行的程序
{
//		Uasrt_Show();

	PID_Change();
//	Oled_Show();
}
//**********************************************增加任务处**********************************************//
//void Run_Task_6(){}
//void Run_Task_7(){}
//void Run_Task_8(){}
//void Run_Task_9(){}
//void Run_Task_10(){}
	
