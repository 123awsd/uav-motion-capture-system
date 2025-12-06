#include "rocker.h"
#include <stdlib.h>

#define ROCKER_ADC_X    hadc1
#define Threshold_Max   (4096+2048)/2
#define Threshold_Min   (0+2048)/2
#define CH_TO_Bin(x)  ((x>Threshold_Min)?((x>Threshold_Max)?1:0):-1)  //将通道值转变为 -1  0  1

//摇杆通道值
int16_t  L_ZY_CH   ,  R_QH_CH  ,  R_ZY_CH ;    
u16 L_QH_CH ;
/*
ADC_buffer[0]  对应    右摇杆左右舵
ADC_buffer[1]  对应    右摇杆前后舵
ADC_buffer[2]  对应    左摇杆左右舵
ADC_buffer[3]  对应    左摇杆前后舵
*/
extern  ADC_HandleTypeDef ROCKER_ADC_X;
volatile u16 ADC_buffer[4]={0};

typedef struct {
int8_t Old_Val;
int8_t Now_Val;
int8_t Key_Down_Val;
int8_t Key_Up_Val;
int8_t Key_Long_Val;
__IO uint32_t Key_Long_SetPoit;
}ROCKER_KEY_STR;

ROCKER_KEY_STR  Rocker_Key[4];


void Rocker_Init()
{
	 HAL_ADCEx_Calibration_Start(&ROCKER_ADC_X);//ADC自动校准
     HAL_Delay(200);//延时200ms
     HAL_ADC_Start_DMA(&ROCKER_ADC_X,(uint32_t*)ADC_buffer,4);
	
	
}



/*
L_QH 对应    左摇杆前后舵
L_ZY  对应    左摇杆左右舵
R_QH  对应    右摇杆前后舵
R_ZY  对应    右摇杆左右舵
*/
//无符号通道值
void Get_Rocker_Unsigned_CH(u16 *L_QH_CH ,u16 *L_ZY_CH ,u16 *R_QH_CH ,u16 *R_ZY_CH )//获取摇杆通道值(0~4096)
{
	(*L_QH_CH) = ADC_buffer[L_QH];
	(*L_ZY_CH) = ADC_buffer[L_ZY];
	(*R_QH_CH) = ADC_buffer[R_QH];
	(*R_ZY_CH) = ADC_buffer[R_ZY];
}
//有符号的通道值
void Get_Rocker_SignedCH(int16_t *L_QH_CH ,int16_t *L_ZY_CH ,int16_t *R_QH_CH ,int16_t *R_ZY_CH )//获取摇杆通道值(-2048~2048)
{
	(*L_QH_CH) = ADC_buffer[L_QH] - 2048;
	(*L_ZY_CH) = ADC_buffer[L_ZY] - 2048;
	(*R_QH_CH)= ADC_buffer[R_QH] - 2048;
	(*R_ZY_CH) = ADC_buffer[R_ZY] - 2048;
}
//油门为无符号通道值，其他为有符号通道值
void Get_Rocker_FlyCH(uint16_t *L_QH_CH ,int16_t *L_ZY_CH ,int16_t *R_QH_CH ,int16_t *R_ZY_CH )
{
//	(*L_QH_CH) = ADC_buffer[L_QH];
	(*L_QH_CH) = 4096-ADC_buffer[L_QH];//油门反相
	
	(*L_ZY_CH) = ADC_buffer[L_ZY] - 2048;
	
//	(*R_QH_CH)= ADC_buffer[R_QH] - 2048;
	(*R_QH_CH)= 4096-ADC_buffer[R_QH] - 2048;	//pitch反相
	
	(*R_ZY_CH) = ADC_buffer[R_ZY] - 2048;
}
//带死区的遥控器读取
#define ROCKER_DEADZONE  150  // 摇杆死区大小，可根据需要调整
void Get_Rocker_FlyCH_with_death(uint16_t *L_QH_CH ,int16_t *L_ZY_CH ,int16_t *R_QH_CH ,int16_t *R_ZY_CH)
{
    // 读取并计算
    (*L_QH_CH) = 4096 - ADC_buffer[L_QH];           // 油门反相（无符号，不加死区）
    (*L_ZY_CH) = ADC_buffer[L_ZY] - 2048;           // 左摇杆左右（居中）
    (*R_QH_CH) = (4096 - ADC_buffer[R_QH]) - 2048;  // 右摇杆前后反相
    (*R_ZY_CH) = ADC_buffer[R_ZY] - 2048;           // 右摇杆左右

    // ===== 死区处理 =====
    if (abs(*L_ZY_CH) < ROCKER_DEADZONE) (*L_ZY_CH) = 0;
    else if (*L_ZY_CH > 0) (*L_ZY_CH) -= ROCKER_DEADZONE;
    else (*L_ZY_CH) += ROCKER_DEADZONE;

    if (abs(*R_QH_CH) < ROCKER_DEADZONE) (*R_QH_CH) = 0;
    else if (*R_QH_CH > 0) (*R_QH_CH) -= ROCKER_DEADZONE;
    else (*R_QH_CH) += ROCKER_DEADZONE;

    if (abs(*R_ZY_CH) < ROCKER_DEADZONE) (*R_ZY_CH) = 0;
    else if (*R_ZY_CH > 0) (*R_ZY_CH) -= ROCKER_DEADZONE;
    else (*R_ZY_CH) += ROCKER_DEADZONE;
}

void Rocker_Key_Scan()
{
	for(int i = 0;i<4;i++)
	{
		Rocker_Key[i].Now_Val = CH_TO_Bin(ADC_buffer[i]);
		if(i == L_ZY)Rocker_Key[i].Now_Val = -Rocker_Key[i].Now_Val;//让左右摇杆方向一致
		Rocker_Key[i].Key_Down_Val = Rocker_Key[i].Now_Val&(Rocker_Key[i].Now_Val^Rocker_Key[i].Old_Val);
		Rocker_Key[i].Key_Up_Val = ~Rocker_Key[i].Now_Val&(Rocker_Key[i].Now_Val^Rocker_Key[i].Old_Val);
		Rocker_Key[i].Old_Val =Rocker_Key[i].Now_Val;
	}
	
	
	
}

void Rocker_Debug()
{
//	printf("%d,%d,%d,%d\r\n",ADC_buffer[0],ADC_buffer[1],ADC_buffer[2],ADC_buffer[3]);
//	printf("%d,%d,%d,%d\r\n",CH_TO_Bin(ADC_buffer[0]),CH_TO_Bin(ADC_buffer[1]),CH_TO_Bin(ADC_buffer[2]),CH_TO_Bin(ADC_buffer[3]));
//	printf("%d,%d,%d,%d\r\n",Rocker_Key[R_ZY].Key_Down_Val,Rocker_Key[R_QH].Key_Down_Val,Rocker_Key[L_ZY].Key_Down_Val,Rocker_Key[L_QH].Key_Down_Val);
}

/**
  * @brief  摇杆按键
  * @param  输入按键模式@KEY_STATE_ENUM，和按键编号@ROCKER_Key_Num
  * @retval 返回@ROCKER_Key_Val
  * @timel 
  */

int8_t Get_Rocker_Key_Val(u8 mode , u8 Rocker_Num)
{
	switch(mode)
	{
		case Rocker_Down_Mode:
		{
			return Rocker_Key[Rocker_Num].Key_Down_Val;

		}
		case Rocker_Up_Mode:
		{
			return Rocker_Key[Rocker_Num].Key_Up_Val;

		}
		case Rocker_Long_Mode:
		{
			return Rocker_Key[Rocker_Num].Key_Long_Val;

		}
	}
	
	return 0;
}

