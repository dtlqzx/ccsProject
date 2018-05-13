#include <msp430g2253.h>
#include <msp430.h>
#include "stdint.h"

/*
 * @fn:		uint16_t Max(uint16_t *numptr,uint16_t num)
 * @brief:	查找最大值
 * @para:
 * @return:	最大值
 * @comment:
 */
uint16_t Max(uint16_t *numptr,uint16_t num)
{
	uint16_t cnt = 0;
	uint16_t max = 0;
	for(cnt = 0;cnt < num;cnt ++)
	{
		if(numptr[cnt] > max){
			max = numptr[cnt];
		}
	}
	return max;
}
/*
 * @fn:		uint16_t Min(uint16_t *numptr,uint16_t num)
 * @brief:	查找最小值
 * @para:
 * @return:	最小值
 * @comment:
 */
uint16_t Min(uint16_t *numptr,uint16_t num)
{
	uint16_t cnt = 0;
	uint16_t min = 0;
	min = numptr[0];
	for(cnt = 0;cnt < num;cnt ++)
	{
		if(numptr[cnt] < min){
			min = numptr[cnt];
		}
	}
	return min;
}
/*
 * @fn:		uint16_t Average(uint16_t *datptr)
 * @brief:	计算平均值
 * @para:	none
 * @return:	none
 * @comment:
 */
uint16_t Average(uint16_t *datptr)
{
	uint32_t sum = 0;
	uint8_t cnt = 0;
	for(cnt = 0;cnt < 50;cnt ++)
	{
		sum += *(datptr + cnt);
	}
	return (uint16_t)(sum / 50);
}
float SqrtByNewton(float x)
{
	const float eps = 0.000001;
    float val = x;//最终
    float last;//保存上一个计算的值
    do
    {
        last = val;
        val =(val + x/val) / 2;
    }while((val-last) < -eps || (val-last) > eps);
    return val;
}
