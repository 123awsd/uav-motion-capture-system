#include "my_math.h"

/**
  * @brief  牛顿迭代法求平方根
  * @param  
  * @retval 
  */

float InvSqrt(float x)
{
	float xhalf = 0.5f*x;
	int i = *(int*)&x; // get bits for floating VALUE 
	i = 0x5f375a86- (i>>1); // gives initial guess y0
	x = *(float*)&i; // convert bits BACK to float
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy

	return 1/x;
}

/**
  * @brief  泰勒级数算sin(来自chatgpt)
  * @param  
  * @retval 
  * @timel 
  */
float fast_sin(float x) 
{
    x = fmod(x, 2 * My_Pi); // 将角度映射到区间 [0, 2π]
    
    float result = 0.0;
    float term = x;
    float sign = 1.0;
    
    for (int i = 1; i <= 10; i += 2) {
        result += sign * term;
        term *= (x * x) / (i * (i + 1));
        sign *= -1.0;
    }
    
    return result;
}
//cos(x) = sin(x+My_Pi/2)
float fast_cos(float x)
{
	return fast_sin(x + My_Pi_2);
}

//反正切麦克劳林展开式 阶数越高，值越准确   70°以内是准确的
//http://www.zybang.com/question/246f9997776f7d5cc636b10aff27a1cb.html
float fast_atan(float x)  //  (-1 , +1)    6? ?? 0.002958 
{
	float t = x;
	float result = 0;
	float X2 = x * x;
	unsigned char cnt = 1;
	do
	{
		result += t / ((cnt << 1) - 1);
		t = -t;
		t *= X2;
		cnt++;
	}while(cnt <= 6);//5??
	return result;
}
/**
  * @brief  角度0~360限制
  * @param  
  * @retval 
  * @timel 
  */

float angle_360_lim(float angle)
{
	if(angle >360 )angle = 0;
	else if(angle <0 )angle = 360;
	return angle;
}


