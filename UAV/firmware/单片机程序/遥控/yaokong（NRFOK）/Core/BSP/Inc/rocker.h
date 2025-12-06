#ifndef __ROCKER_H__
#define __ROCKER_H__
#include "main.h"
#define u8		uint8_t
#define u16		uint16_t
#define uint	uint32_t
typedef enum {
	Rocker_Down_Mode = 0,//按下
	Rocker_Up_Mode,//松开
	Rocker_Long_Mode,//长按
}ROCKER_Key_Get_Mode_ENUM;

typedef enum {
	R_ZY = 0,//按下
	R_QH = 1,//松开
	L_ZY = 2,//长按
	L_QH = 3,
}ROCKER_Key_Num;
typedef enum {
	Rocker_Key_Qian = 1,//按下
	Rocker_Key_Zuo = 1,//松开
	Rocker_Key_Hou = -1,//长按
	Rocker_Key_You = -1,
}ROCKER_Key_Val;

extern int16_t  L_ZY_CH   ,  R_QH_CH  ,  R_ZY_CH ;    
extern u16 L_QH_CH ;

void Rocker_Init(void);
void Rocker_Debug(void);
void Rocker_Key_Scan(void);
void Get_Rocker_FlyCH(uint16_t *L_QH_CH ,int16_t *L_ZY_CH ,int16_t *R_QH_CH ,int16_t *R_ZY_CH );
void Get_Rocker_Unsigned_CH(u16 *L_QH_CH ,u16 *L_ZY_CH ,u16 *R_QH_CH ,u16 *R_ZY_CH );//获取摇杆通道值(0~4096)
void Get_Rocker_Signed_CH(int16_t *L_QH_CH ,int16_t *L_ZY_CH ,int16_t *R_QH_CH ,int16_t *R_ZY_CH );//获取摇杆通道值(-2048~2048)
void Get_Rocker_FlyCH_with_death(uint16_t *L_QH_CH ,int16_t *L_ZY_CH ,int16_t *R_QH_CH ,int16_t *R_ZY_CH);//带死区遥控器通道值获取
int8_t Get_Rocker_Key_Val(uint8_t mode , uint8_t Rocker_Num);






#endif

