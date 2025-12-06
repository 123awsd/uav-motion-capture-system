#ifndef __TASK_DISPATCH_H__
#define __TASK_DISPATCH_H__
#include "main.h"

#define BASIC_TIME   		1   //计数值增加基础时间为1ms   可根据定时器中断时间修改
#define TASK_MAX_NUM  		5	//最大任务数
#define TASK_PRIORITY_MAX	250 //最大优先级数量

typedef enum {
	Self_Mode = 0,//主任务模式
	Other_Mode,//从任务模式

}task_run_fater_Mode_ENUM;
typedef enum {
	Handoff_Self= 0,//切换到主任务
	Handoff_Other,//切换到从任务

}task_run_Handoff_Mode_ENUM;//主从模式之间切换切换

typedef struct TASK_NODE{
//	char name[20];
	uint8_t  						 task_code_num; //任务代号
	volatile uint16_t				 task_cnt ;//计数值
	uint16_t 						 task_FQR ;//任务运算频率
	bool 							 task_Enable_Flag;//任务使能标志
	bool     						 task_IDIE_Flag;//任务空闲标志
	uint8_t 						 task_priority;//任务优先级
	bool                             task_run_after_flag;
	volatile uint32_t                task_run_after_Setpoit;//任务延时运行计时
}TASK_INF_TYPEDEF;

/*
注意事项：

关于优先级——
1.优先级值越低代表优先级越高；
2.当两个任务优先级相同时，创建任务的顺序决定了任务的优先级，越后创建优先级越高

关于使用  ——
1.先调用Tsak_Init();完成初始化
2.创建任务变量 TASK_INF_TYPEDEF  Task_x;
3.调用函数Add_Task( uint16_t Task_FRQ ,uint8_t Task_priority ,bool task_IsEnable);依次传入 任务频率， 任务优先级， 是否执行该任务；来创建任务
4.将Task_Cnt_Inc(void);   Task_Cnt_Check(void);两个函数放在定时器中，并根据定时器时间修改宏定义@BASIC_TIME 来修改基础时间
5.使用TASK_Run允许任务运行  TASK_Stop停止运行所有任务
6.Enable_Task() 解挂任务 ，DisEnable_Task挂起任务

关于增加任务上限
1.修改宏定义@TASK_MAX_NUM 数值
2.在增加任务处添加任务




*/

void Tsak_Init(void);//初始化
TASK_INF_TYPEDEF  Add_Task( uint16_t Task_FRQ ,uint8_t Task_priority ,bool task_IsEnable);//添加任务
void  Enable_Task( TASK_INF_TYPEDEF Task);//使能任务
void  DisEnable_Task( TASK_INF_TYPEDEF Task);//失能任务
void Task_Cnt_Inc(void);//放在定时器中，根据定时时间修改BASIC_TIME
void Task_Cnt_Check(void);//放在定时器中，调用任务函数
void TASK_Debug(void);
void TASK_Run(void);//任务允许运行
void TASK_Stop(void);//任务不允许运行


//在任务里延时，并不阻塞其他任务，相当于可以在任务里用while(一段时间)；//需在两个任务里调用
void task_run_fater_ms(TASK_INF_TYPEDEF * Self_Task , TASK_INF_TYPEDEF * Other_Task,uint16_t Time ,uint8_t mode );
void task_run_Handoff(TASK_INF_TYPEDEF * Self_Task , TASK_INF_TYPEDEF * Other_Task,uint8_t mode );//主从任务切换
#endif

