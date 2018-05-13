#include <msp430g2253.h>
#include <msp430.h>
#include "stdint.h"

/********************************************

            LCD液晶操作函数

*******************************************/

void LCD_init_first(void)         //LCD1602液晶初始化函数（热启动）
{
        delay_nms(500);
     LCD_DATA_DDR|=LCD_DATA;   //数据口方向为输出
     LCD_EN_DDR|=LCD_EN;       //设置EN方向为输出
     LCD_RS_DDR|=LCD_RS;       //设置RS方向为输出
//   P2DIR |= BIT7|BIT6|BIT5|BIT4;
//   P1DIR |= BIT7;
//   P1DIR |= BIT6;
        delay_nms(50);
        LCD_write_command(0x30);
        delay_nms(50);
        LCD_write_command(0x30);
        delay_nms(5);
        LCD_write_command(0x30);
        delay_nms(500);

}

/*****************************************
*
*             LCD1602液晶初始化函数
*
****************************************/
void LCD_init(void)
{
delay_nms(500);
	LCD_DATA_DDR|=LCD_DATA;   //数据口方向为输出
	LCD_EN_DDR|=LCD_EN;       //设置EN方向为输出
	LCD_RS_DDR|=LCD_RS;       //设置RS方向为输出
//   P2DIR |= BIT7|BIT6|BIT5|BIT4;
//   P1DIR |= BIT7;
//   P1DIR |= BIT6;
delay_nms(500);

LCD_write_command(0x28);  //4位数据接口
delay_nms(50);
LCD_write_command(0x28);  //4位数据接口
delay_nms(50);
    LCD_write_command(0x28);  //4位数据接口
delay_nms(50);
    LCD_en_write2();
    delay_nms(50);
LCD_write_command(0x28); //4位数据接口
delay_nms(500);
LCD_write_command(0x01); //清屏
LCD_write_command(0x0c); //显示开，关光标，不闪烁
    LCD_write_command(0x06); //设定输入方式，增量不移位
delay_nms(50);


}

/*****************************************
*
*             液晶使能上升沿
*
****************************************/

void LCD_en_write1(void)
{
    LCD_EN_PORT&=~LCD_EN;
    //P1OUT &= ~BIT7;
    delay_nus(10);
    LCD_EN_PORT|=LCD_EN;
    //P1OUT |= BIT7;
}

/*****************************************
*
*             液晶使能下降沿
*
****************************************/
void LCD_en_write2(void)
{
   LCD_EN_PORT|=LCD_EN;
   //P1OUT |= BIT7;
   delay_nus(10);
   LCD_EN_PORT&=~LCD_EN;
   //P1OUT &= ~BIT7;
}

/*****************************************
*
*               写指令函数
*
****************************************/
void LCD_write_command(unsigned char command)
{
   delay_nus(16);
   P2SEL &= ~BIT6;
   P2SEL &= ~BIT7;
   LCD_RS_PORT&=~LCD_RS; //RS=0
   //P1OUT &= ~BIT6;
   LCD_en_write1();
   LCD_DATA_PORT&=0X0f; //清高四位
   //P2OUT &= 0X0F;
   LCD_DATA_PORT|=command&0xf0; //写高四位
   //P2OUT |= command & 0xf0;

   delay_nus(16);
   LCD_en_write2();
   command=command<<4; //低四位移到高四位
   LCD_en_write1();
   LCD_DATA_PORT&=0x0f; //清高四位
   //P2OUT &= 0X0F;
   LCD_DATA_PORT|=command&0xf0; //写低四位
   //P2OUT |= command & 0xf0;
   LCD_en_write2();
}

/*****************************************
*
*               写数据函数
*
****************************************/
void LCD_write_data(unsigned char data)
{
   delay_nus(16);
   P2SEL &= ~BIT6;
   P2SEL &= ~BIT7;

   LCD_RS_PORT|=LCD_RS;      //RS=1
   //P1OUT |= BIT6;
   LCD_en_write1();          //E上升沿
   LCD_DATA_PORT&=0X0f;      //清高四位
   //P2OUT |= 0x0f;
   LCD_DATA_PORT|=data&0xf0; //写高四位
   //P2OUT |= data & 0xf0;
   delay_nus(16);
   LCD_en_write2();
   data=data<<4;             //低四位移到高四位
   LCD_en_write1();
   LCD_DATA_PORT&=0X0f;      //清高四位
   //P2OUT |= 0x0f;
   LCD_DATA_PORT|=data&0xf0; //写低四位
   LCD_en_write2();
   //P2OUT |= data & 0xf0;

}

/*****************************************
*
*               写地址函数
*
****************************************/
void LCD_set_xy( unsigned char x, unsigned char y )
{
   unsigned char address;
   if (y == 0) address = 0x80 + x;
   else address = 0xc0 + x;
   LCD_write_command( address);
}

/*****************************************
*
*LCD在任意位置写字符串，列x=0~15,行y=0,1
*
****************************************/
void LCD_write_string(unsigned char X,unsigned char Y,unsigned char *s)
{
LCD_set_xy( X, Y ); //写地址
    while (*s)          //写显示字符
    {
      LCD_write_data( *s );
      s++;
    }
}

/*****************************************
*
*     LCD在任意位置写字符,列x=0~15,行y=0,1
*
****************************************/

void LCD_write_char(unsigned char X,unsigned char Y,unsigned char data)
{
   LCD_set_xy( X, Y );//写地址
   LCD_write_data( data);
}

/*****************************************
*
*               1us延时函数
*
****************************************/

void delay_1us(void)
{
   asm("nop");
}

/*****************************************
*
*               N us延时函数
*
****************************************/
void delay_nus(unsigned int n)
{
   unsigned int i;
   for (i=0;i<n;i++)
   delay_1us();
}

/*****************************************
*
*               1ms延时函数
*
****************************************/
void delay_1ms(void)
{
   unsigned int i;
   for (i=0;i<1140;i++);
}

/*****************************************
*
*               N ms延时函数
*
****************************************/
void delay_nms(unsigned int n)
{
   unsigned int i=0;
   for (i=0;i<n;i++)
   delay_1ms();
}
void LCD_stuNum(void)
{
	LCD_write_string(0,0,"2015211021");
	delay_nms(10);
	LCD_write_string(0,1,"Zsq && Zx");
}
void LCD_Type_Vpp(char* type,float vpp)
{
	uint8_t charbuff[] = {'#','#','#','#','.','#','#','#',' ',' ','m','V','p','p','\0'};
	uint16_t interger = (uint16_t)vpp;
	uint16_t pointNum = (uint16_t)((vpp - interger)*1000);
	charbuff[0] = interger / 1000 % 10 + '0';
	charbuff[1] = interger / 100 % 10 + '0';
	charbuff[2] = interger / 10 % 10 + '0';
	charbuff[3] = interger / 1 % 10 + '0';
	/***************************************/
	charbuff[5] = pointNum / 100 % 10 + '0';
	charbuff[6] = pointNum / 10 % 10 + '0';
	charbuff[7] = pointNum / 1 % 10 + '0';

	LCD_write_string(0,0,type);
	LCD_write_string(0,1,charbuff);



}
void LCD_Freq_Vrms(uint16_t freq,float vrms)
{
// send freq
	uint8_t buff[8] = {'#','#','#','#','#','H','z','\0'};
	uint8_t cnt = 0;
	for(cnt = 0;cnt < 5;cnt ++)
	{
		buff[4 - cnt] = (uint8_t)(freq % 10 + '0');
		freq /= 10;
	}
//	send vrms
	uint8_t charbuff[] = {'#','#','#','#','.','#','#','#',' ',' ','m','V','r','m','s','\0'};
	uint16_t interger = (uint16_t)vrms;
	uint16_t pointNum = (uint16_t)((vrms - interger)*1000);
	charbuff[0] = interger / 1000 % 10 + '0';
	charbuff[1] = interger / 100 % 10 + '0';
	charbuff[2] = interger / 10 % 10 + '0';
	charbuff[3] = interger / 1 % 10 + '0';
	/***************************************/
	charbuff[5] = pointNum / 100 % 10 + '0';
	charbuff[6] = pointNum / 10 % 10 + '0';
	charbuff[7] = pointNum / 1 % 10 + '0';

	LCD_write_string(0,0,buff);
	LCD_write_string(0,1,charbuff);
}
