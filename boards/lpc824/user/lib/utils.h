#ifndef  __UTILS_H__
#define  __UTILS_H__
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

typedef struct
{
    bool up;
    uint32_t start;
    uint32_t value;
}utils_timer_t;


#ifndef   IS_POWER_OF_TWO
#define   IS_POWER_OF_TWO(A)   (((A) != 0) && ((((A) - 1) & (A)) == 0))
#endif

#ifndef   MIN
#define   MIN(A,B)             ((A) > (B) ? (B) :(A))
#endif

#ifndef   MAX
#define   MAX(A,B)             ((A) > (B) ? (A) :(B))
#endif

#ifndef  UTILS_ASSERT
#define  UTILS_ASSERT(x)                            \
do {                                                \
    if ((void *)(x) == (void *)0) {                 \
        while(1);                                   \
    }                                               \
}while (0)                                         
#endif



/*
* @brief 数组转成HEX字符串
* @param
* @param
* @return 
* @note
*/
int dump_hex_str(const char *src,char *dst,uint16_t src_len);
/* 函数：utils_timer_init
*  功能：自定义定时器初始化
*  参数：timer 定时器指针
*  参数：timeout 超时时间
*  参数：up 定时器方向-向上计数-向下计数
*  返回: 0：成功 其他：失败
*/ 
int utils_timer_init(utils_timer_t *timer,uint32_t timeout,bool up);
/* 函数：utils_timer_value
*  功能：定时器现在的值
*  返回：>=0：现在时间值 其他：失败
*/ 
uint32_t utils_timer_value(utils_timer_t *timer);   

/* 函数：utils_atof
*  功能：字符串转浮点
*  返回：浮点数
*/   
double utils_atof(char *s);
   
#endif