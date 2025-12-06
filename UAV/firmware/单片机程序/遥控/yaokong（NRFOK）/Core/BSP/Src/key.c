#include "key.h"


//记得调用key_scan;
typedef struct 
{
	uint8_t Key_Down,Key_Up,Key_Long,Key_LongLong,Key_val,Key_Old;
	volatile uint32_t KEY_SetPoit ;
	
}KEY_TYPDEF;
static KEY_TYPDEF KEY;

uint8_t Get_Key_State()
{
//	
	if( Read_KEY_Side_L == Key_Down_Level) return  KEY_Side_L;
	if(Read_Read_KEY_Side_R  == Key_Down_Level) return  KEY_Side_R;
	if(Read_KEY_Front_U == Key_Down_Level) return  KEY_Front_U;
	if( Read_KEY_Front_D == Key_Down_Level) return  KEY_Front_D;
	if(Read_KEY_Front_R  == Key_Down_Level) return  KEY_Front_R;
	if(Read_KEY_Front_L == Key_Down_Level) return  KEY_Front_L;
	if(Read_KEY_Rocker_L  == Key_Down_Level) return  KEY_Rocker_R;
	if(Read_KEY_Rocker_R == Key_Down_Level) return  KEY_Rocker_L;
	return NO_KEY;
}
void Key_Scan()
{
	KEY.Key_val = Get_Key_State();
	
	KEY.Key_Down = KEY.Key_val & (KEY.Key_val ^ KEY.Key_Old);
	KEY.Key_Up = ~KEY.Key_val & (KEY.Key_val ^ KEY.Key_Old);
	KEY.Key_Old = KEY.Key_val;
	if(KEY.Key_Down)
	{
		KEY.KEY_SetPoit = uwTick;
	}
	KEY.Key_Long = NO_KEY;
	KEY.Key_LongLong = NO_KEY;
	if(KEY.Key_Up)
	{
		
		if((uwTick - KEY.KEY_SetPoit) >= KeyLongLong_Time )
		{
			KEY.Key_LongLong = KEY.Key_Up;
			KEY.Key_Up = NO_KEY;
		}
		else if( (uwTick - KEY.KEY_SetPoit) >= KeyLong_Time)
		{
			KEY.Key_Long = KEY.Key_Up;
			KEY.Key_Up = NO_KEY;
		}
		else
		{
			
		}
		
	}
}
//mode选择为@Key_Get_Mode_ENUM
uint8_t  Get_Key_Val(uint8_t mode)
{
	if(mode == Down_Mode)
	{
		return KEY.Key_Down;
	}
	if(mode == Up_Mode)
	{
		return KEY.Key_Up;
	}
	if(mode == Long_Mode)
	{
		return KEY.Key_Long;
	}
	if(mode == LongLong_Mode)
	{
		return KEY.Key_LongLong;
	}
	 return NO_KEY;
}






