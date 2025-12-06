#include "tack_dispatch.h"
typedef struct {
	TASK_INF_TYPEDEF  					Task[TASK_MAX_NUM];
	uint8_t 							Task_Num;//当前任务数
	uint8_t                             Task_priori_Sequence[TASK_MAX_NUM];//按任务优先级排列的顺序，越靠前对应下标的任务优先级越高
	bool                                Task_Run;//任务使能标志位//可以关闭所有任务
}ALL_TASK_TYPEDEF;

typedef enum{
	TASK1 = 0,
	TASK2,
	TASK3,
	TASK4,
	TASK5,
	TASK6,
	TASK7,
	TASK8,
	TASK9,
	TASK10
}TASK_ENUM;


ALL_TASK_TYPEDEF					 All_Task;

void Tsak_Init()
{	
	All_Task.Task_Num = 0;
	All_Task.Task_Run = false;
	for(int i = 0 ; i<TASK_MAX_NUM;i++)
	{
		All_Task.Task[i].task_cnt = 0;
		All_Task.Task_priori_Sequence[i] = TASK_MAX_NUM;//让初始状态等于不可能出现的下标，防止以后误判 
		All_Task.Task[i].task_priority = 100; //初始优先级为最低
		All_Task.Task[i].task_run_after_flag = true;//任务延时运行标志位置位
		All_Task.Task[i].task_run_after_Setpoit = 0;
//		printf("%d ,%d,%d,",All_Task.Task[i].task_priority,All_Task.Task[i].task_priority,All_Task.Task[i].task_priority);
	}
}
void Insert_Task(TASK_INF_TYPEDEF  Task)//安装任务的优先级在合适位置插入任务  优先级值小的在前	
{
	uint8_t temp = 0; 
	int i;
	All_Task.Task[All_Task.Task_Num]		 = Task;//将任务存在allTask结构体里
	if(All_Task.Task_Num == 0)//先前没有任务
	{
		All_Task.Task_priori_Sequence[0]	 = Task.task_code_num;
	}
	else
	{
		for(i = 0 ;i<All_Task.Task_Num;i++)
		{
			temp = All_Task.Task_priori_Sequence[i];
//			printf(" %d %d %d  \r\n",i,All_Task.Task[temp].task_priority ,Task.task_priority );
			if(All_Task.Task[temp].task_priority >= Task.task_priority )//找到要插入的位置
			{
//				//
//				printf(" CK %d  %d \r\n",All_Task.Task_Num,i);
//				
//				//
				for(int j = All_Task.Task_Num ; j>=i;j--)
				{
					All_Task.Task_priori_Sequence[j+1] = All_Task.Task_priori_Sequence[j];//将插入位置后的编码整体后移
				}
				All_Task.Task_priori_Sequence[i] = Task.task_code_num;
//				All_Task.Task[i] =  Task;
				break;//结束循环
			}
		}
		if( i == All_Task.Task_Num )
		{
			All_Task.Task_priori_Sequence[All_Task.Task_Num] = Task.task_code_num;
//				All_Task.Task[All_Task.Task_Num] =  Task;
		}
		
		
	}
//	printf("顺序%d %d %d %d\r\n",All_Task.Task_priori_Sequence[0] , All_Task.Task_priori_Sequence[1] , All_Task.Task_priori_Sequence[2],All_Task.Task_priori_Sequence[3]);
	All_Task.Task[All_Task.Task_Num] =  Task;
	All_Task.Task_Num ++;//任务数加1
//	for( i = 0 ; )
	
	
}	
TASK_INF_TYPEDEF  Add_Task( uint16_t Task_FRQ ,uint8_t Task_priority ,bool task_IsEnable)
{
	TASK_INF_TYPEDEF  Task_temp;
	
	Task_temp.task_FQR 				= Task_FRQ ;//任务频率
	Task_temp.task_code_num 		= All_Task.Task_Num;//任务编号
	Task_temp.task_priority			= Task_priority;//优先级
	Task_temp.task_Enable_Flag 		= task_IsEnable;//是否使能任务
	Task_temp.task_run_after_flag   = true;//任务延时运行标志位置位
	Task_temp.task_run_after_Setpoit= 0;//记录当前时间为0
	Insert_Task(Task_temp);
	
	return Task_temp;//返回任务信息结构体
}
void  DisEnable_Task( TASK_INF_TYPEDEF Task)
{
	uint8_t temp;
	temp = Task.task_code_num;
	All_Task.Task[temp].task_Enable_Flag = false;
}
void  Enable_Task( TASK_INF_TYPEDEF Task)
{
	uint8_t temp;
	temp = Task.task_code_num;
	All_Task.Task[temp].task_Enable_Flag = true;
}

//放定时器里
void Task_Cnt_Inc()
{
	if(All_Task.Task_Run == true)
	{
		for(int i = 0 ; i<All_Task.Task_Num;i++)//计数值增加
		{
			All_Task.Task[i].task_cnt += BASIC_TIME;
		}
	}
}
void Task_Cnt_Check()//检测计数值
{
	uint8_t temp ,Past_task_priori = 255;//上一个执行任务的优先级初始值为一个不可能值
	if(All_Task.Task_Run == true)
	{
		for(int i = 0 ; i<All_Task.Task_Num;i++ )//优先级由高到低扫描
		{
			temp = All_Task.Task_priori_Sequence[i] ;//获取任务编号

			if(All_Task.Task[temp].task_Enable_Flag == true)//任务未被挂起
			{
				if(All_Task.Task[temp].task_cnt  >= (1000/All_Task.Task[temp].task_FQR)  )
				{
					if(Past_task_priori == 255   ||   Past_task_priori == All_Task.Task[temp].task_priority)//同一优先级的任务同一批运行(防止所有任务加起来等于1000时，低优先级任务无法执行)
					{
					}
					else
					{
						break;
					}
					All_Task.Task[temp].task_cnt = 0;
					switch(temp)
					{
						case TASK1:
							Run_Task_1();
							break;
						case TASK2:
							Run_Task_2();
							break;
						case TASK3:
							Run_Task_3();
							break;
						case TASK4:
							Run_Task_4();
							break;
						case TASK5:
							Run_Task_5();
							break;
//***********************************************增加任务数处******************************************************************/						
//						case TASK6:
//							Run_Task_3();
//							break;
//						case TASK7:
//							Run_Task_4();
//							break;
//						case TASK8:
//							Run_Task_5();
//							break;
//						case TASK9:
//							Run_Task_3();
//							break;
//						case TASK10:
//							Run_Task_4();
//							break;
					}
					//尝试函数指针？？？？
					// break;//运行完从头开始扫描
					
					Past_task_priori = All_Task.Task[temp].task_priority;
				}
			}
		}
	}
}
void TASK_Run()
{
	All_Task.Task_Run = true;
//	printf(" %d \r\n",All_Task.Task[0].task_Enable_Flag);
}
void TASK_Stop()
{
	All_Task.Task_Run = false;
}
void TASK_Debug()
{
//	printf("%d %d %d %d %d\r\n",All_Task.Task_priori_Sequence[0] , All_Task.Task_priori_Sequence[1] , All_Task.Task_priori_Sequence[2], All_Task.Task_priori_Sequence[3], All_Task.Task_priori_Sequence[4]);
//	printf("%d %d %d \r\n",All_Task.Task[0].task_cnt ,All_Task.Task[1].task_cnt ,All_Task.Task[2].task_cnt  );
}





//功能描述
//用两个任务互相解挂，挂起来实现，可以实现在这个任务挂起的时候跑一些其他的内容，等其他内容跑完后重新回归该任务，需要一个主任务和一个从任务
//主任务来运行逻辑，当需要停下来运行其他内容时，调用从任务，从任务一开始需为挂起状态
//变量：
//Self_Task：本身任务   Other_Task：另一个任务
//Time为延时时间，
//mode：选择在主任务调用还是从任务调用 可以是@task_run_fater_Mode_ENUM
void task_run_fater_ms(TASK_INF_TYPEDEF *Self_Task , TASK_INF_TYPEDEF *Other_Task,uint16_t Time ,uint8_t mode )
{	
	switch(mode)
	{
		case Self_Mode:
		{
			if(Self_Task->task_run_after_flag)
			{
				Self_Task->task_run_after_flag = false;
				Self_Task->task_run_after_Setpoit = uwTick;//记录当前时间
				DisEnable_Task(*Self_Task);//失能主任务
				Enable_Task(*Other_Task);//使能本任务
			}
			break;
		}
		case Other_Mode:
		{
			if(!Self_Task->task_run_after_flag)
			{
				printf("%d , %d, %d\r\n",uwTick,Self_Task->task_run_after_Setpoit,uwTick - Self_Task->task_run_after_Setpoit);
				if((uwTick - Self_Task->task_run_after_Setpoit) > Time)
				{
					Self_Task->task_run_after_flag = true;
					DisEnable_Task(*Other_Task);//失能从任务
					Enable_Task(*Self_Task);//使能本任务
					
				}
			}
			break;
		}
	}
}
//主从任务之间切换
//mode 可以是@task_run_Handoff_Mode_ENUM
void task_run_Handoff(TASK_INF_TYPEDEF *Self_Task , TASK_INF_TYPEDEF *Other_Task ,uint8_t mode )
{	
	switch(mode)
	{
		case Handoff_Self:
		{
			DisEnable_Task(*Other_Task);//失能从任务
			Enable_Task(*Self_Task);//使能本任务
			break;
		}
		case Handoff_Other:
		{
			DisEnable_Task(*Self_Task);//失能主任务
			Enable_Task(*Other_Task);//使能本任务
			break;
		}
	}
}


//*********************************************Tim1中断回调函数（1ms）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim->Instance  == TIM1 )
	{
		Task_Cnt_Inc();
		Task_Cnt_Check();
	}

}



