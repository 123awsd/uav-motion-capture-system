#include "task.h"

//无线接收define
#define RX_Mode  1
#define TX_Mode  0

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
	
	Task_1 = Add_Task( 100 ,2 ,true); //创建任务1 频率，优先级，是否执行  
	Task_2 = Add_Task( 200 ,1 ,true); //创建任务2
	Task_3 = Add_Task( 50 ,5 ,true); //创建任务3 
	Task_4 = Add_Task( 50 ,5 ,true); //创建任务4 
	Task_5 = Add_Task( 50 ,7 ,true); //创建从任务 来辅助其他任务
//	Task_6 = Add_Task( 4 ,1 ,true); //创建任务6  
//	Task_7 = Add_Task( 4 ,4 ,true); //创建任务7 
//	Task_8 = Add_Task( 4 ,10 ,true); //创建任务8
//	Task_9 = Add_Task( 4 ,1 ,true); //创建任务9  
//	Task_10 = Add_Task( 4 ,1 ,true); //创建任务10 
	
	TASK_Run();
} 

//NRF接收发送的数组
u8 NRF_RX_Buff[32];//发送数组
u8 NRF_TX_Buff[32];//接收数组
void Run_Task_1()//数据发送
{
	
    Data_Send_To_Body();
					
				
}

void Run_Task_2()//数据接受
{
    Data_Receive_Form_Body();
}

void Run_Task_3()//按键任务
{

	Key_Task();
	Rocker_Task();

}

void Run_Task_4()//按键蜂鸣器提示
{

	Beep_Task();
}

void Run_Task_5()//
{
//	Oled_Show();
//	printf("111");
}


void While_Loop()//放在while(1)运行的程序
{
//		Uasrt_Show();
	Oled_Show();
}
//**********************************************增加任务处**********************************************//
//void Run_Task_6(){}
//void Run_Task_7(){}
//void Run_Task_8(){}
//void Run_Task_9(){}
//void Run_Task_10(){}
	
