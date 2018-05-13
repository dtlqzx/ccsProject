#include <msp430g2253.h>
#include <msp430.h>
#include "stdint.h"





void UARTSendString (uint8_t str[],uint8_t len)
{
  int i = 0;
  for(i = 0; i<len; i++)
  {
    while( UCA0STAT & UCBUSY ){}
    UCA0TXBUF = str[i];
  }
}
void PrintString (uint8_t str[])
{
  uint8_t i = 0;
  for(i = 0; str[i]!='\0'; i++)
  {
    while( UCA0STAT & UCBUSY ){}
    UCA0TXBUF = str[i];
  }
  while( UCA0STAT & UCBUSY ){}
  UCA0TXBUF = '\n';

}
void PrintFreq(float freq)
{
	uint32_t temp = (uint32_t)(freq * 1000);
	uint8_t charbuff[] = {0,0,0,0,0,0,0,0,0};//最大999999.999Hz
	int8_t cnt = 0;
	for(cnt = 8;cnt >= 0;cnt --)
	{
		charbuff[cnt] = (uint8_t)(temp % 10) + '0';
		temp /= 10;
	}
	UARTSendString("freq is ",8);
	UARTSendString(charbuff,6);
	UARTSendString(".",1);
	UARTSendString(charbuff + 6,3);
	UARTSendString("Hz",2);
}
void PrintFloat(float num)
{
	uint8_t charbuff[] = {0,0,0,0,0,0,'.',0,0,0,'\0'};
	uint16_t interger = (uint16_t)num;
	uint16_t pointNum = (uint16_t)((num - interger)*1000);
	uint8_t lenOfInte = 6;
	uint8_t i = 0;
	charbuff[0] = interger / 100000 % 10 + '0';
	charbuff[1] = interger / 10000 % 10 + '0';
	charbuff[2] = interger / 1000 % 10 + '0';
	charbuff[3] = interger / 100 % 10 + '0';
	charbuff[4] = interger / 10 % 10 + '0';
	charbuff[5] = interger / 1 % 10 + '0';
	/***************************************/
	charbuff[7] = pointNum / 100 % 10 + '0';
	charbuff[8] = pointNum / 10 % 10 + '0';
	charbuff[9] = pointNum / 1 % 10 + '0';
	PrintString(charbuff);

}
void Print_Type_Real_Full(uint8_t type, float real,float full)
{
	//Type
	const uint8_t square = 1;
	const uint8_t triangle = 2;
	const uint8_t sin = 3;

	PrintString("\nType:");
	switch(type)
	{
	case sin:PrintString("Sin"); break;
	case square:PrintString("Square");break;
	case triangle:PrintString("Triangle");break;
	default:break;
	}
	PrintString("Real Voltage(mV):");
	PrintFloat(real);
	PrintString("Full Voltage(mV):");
	PrintFloat(full);
}
