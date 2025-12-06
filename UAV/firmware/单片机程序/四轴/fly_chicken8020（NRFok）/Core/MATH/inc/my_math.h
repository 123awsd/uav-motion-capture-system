#ifndef __MY_MATH_H__
#define __MY_MATH_H__
#include "main.h"
#define   My_Pi     	 (3.141592f)
#define   My_Pi_2 		 (1.570796f)//pi/2

#define max(x,y) (x>y?x:y)
#define min(x,y) (x<y?x:y)
#define limit(x,y,z) ((min(x,y))>z?(MY_min(x,y)):(MY_min(max(x,y),z)))   //限幅
#define limitt(x,a,b) (min(max(x,a),b))        //限幅  要求a<b
#define ABS(x) ((x) > 0 ? (x) : -(x))

float InvSqrt(float x);
float fast_sin(float x) ;
float fast_cos(float x);


float angle_360_lim(float angle);//角度0~360限制
#endif

