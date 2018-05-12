/*********************************************************************
*P2.0,P2.1,P2.2--控制电阻高阻、接地
*P2.0--控制 10k ohm
*P2.1--控制 1k  ohm
*P2.2--控制 100 ohm
*P1.0--ADC采样--测电阻
*P2.5--Timer测频--测电容
*P1.4,P1.5,P1.6,P1.7,XIN(P2.6),XOUT(P2.7),GND---控制LCD1602
*
*
*
*
*
*
*
*
*****************************************************************/

#include <intrinsics.h>
#include <msp430g2253.h>
#include <msp430.h>
#include "stdint.h"
/*****************************************************

                                                     端口定义

****************************************************/
#define LCD_EN_PORT P2OUT    //以下2个要设为同一个口
#define LCD_EN_DDR P2DIR
#define LCD_RS_PORT P2OUT    //以下2个要设为同一个口
#define LCD_RS_DDR P2DIR
#define LCD_RW_PORT P2OUT
#define LCD_RW_DDR P2DIR
#define LCD_DATA_PORT P1OUT  //以下3个要设为同一个口
#define LCD_DATA_DDR P1DIR   //一定要用高4位
#define LCD_RS BIT6
#define LCD_RW BIT5
#define LCD_EN BIT7
#define LCD_DATA    BIT7|BIT6|BIT5|BIT4   //4位数据线连接模式
#define ADCBUFSIZE 50
#define LONGTIME 40000000
/************************IO part*******************/
#define DIR2_0 (P2DIR & BIT0)
#define DIR2_1 (P2DIR & BIT1)
#define DIR2_2 (P2DIR & BIT2)
#define OUT2_0 (P2OUT & BIT0)
#define OUT2_1 (P2OUT & BIT1)
#define OUT2_2 (P2OUT & BIT2)

#define P2_0_SET_HIGH P2DIR &= ~BIT0
#define P2_1_SET_HIGH P2DIR &= ~BIT1
#define P2_2_SET_HIGH P2DIR &= ~BIT2
#define P2_0_SET_GND P2DIR |= BIT0; P2OUT &= ~BIT0;
#define P2_1_SET_GND P2DIR |= BIT1; P2OUT &= ~BIT1;
#define P2_2_SET_GND P2DIR |= BIT2; P2OUT &= ~BIT2;

#define P2_0_IS_HIGH (~DIR2_0)
#define P2_1_IS_HIGH (~DIR2_1)
#define P2_2_IS_HIGH (~DIR2_2)
#define P2_0_IS_GND ((DIR2_0) && (~OUT2_0))
#define P2_1_IS_GND ((DIR2_1) && (~OUT2_1))
#define P2_2_IS_GND ((DIR2_2) && (~OUT2_2))

/******************display part************************/
#define SMALLRANGE

/******************全局变量部分*****************************/

//测电压部分变量
volatile uint16_t adcbuff[50] = {0};
int16_t typeBuff[50] = {0};
//uint16_t maxval[50] = {0};
//uint16_t minval[50] = {0};

uint16_t max = 0;
uint16_t min = 65535;
uint16_t vpp = 0;

//测频部分变量
float freq = 0;
uint16_t freqCnt = 0;
uint16_t T = 0;
const uint16_t t_1s = 20;
uint32_t timestamp = 0;//时间戳
uint16_t capvalue_1 = 0;//第一次捕捉值
uint16_t capvalue_2 = 0;//第二次捕捉值
uint32_t timestamp_1 = 0;//第一次时间戳
uint32_t timestamp_2 = 0;//第二次时间戳
uint32_t totaltime = 0;

//输出部分变量
volatile uint8_t displayStatus = 3;
//表达电容部分变量
volatile float Cvalue = 0;


//判断部分变量
const int low = 5,mid = 10,high = 300;
uint8_t lowNum = 0,midNum = 0,highNum = 0;

uint8_t next = 0;


void IniClk(void);
void IniButton(void);
void IniADC();
void IniUART(void);
void IniTimerA1_ct();
void SpiWriteDAC(uint16_t data,uint8_t channel);

uint16_t GetADCValue(void);
void StartADCConvert(void);
uint16_t Max(uint16_t *numptr,uint16_t num);
uint16_t Min(uint16_t *numptr,uint16_t num);
uint16_t Average(uint16_t *datptr);
float SqrtByNewton(float x);
void UARTSendString (uint8_t str[],uint8_t len);
void PrintString (uint8_t str[]);
void PrintFreq(float freq);
void PrintFloat(float num);
void Print_Type_Real_Full(uint8_t type, float real,float full);


void LCD_init_first(void);
void LCD_init(void);
void LCD_en_write1(void);
void LCD_en_write2(void);
void LCD_write_command(unsigned char command);
void LCD_write_data(unsigned char data);
void LCD_set_xy( unsigned char x, unsigned char y );
void LCD_write_string(unsigned char X,unsigned char Y,unsigned char *s);
void LCD_write_char(unsigned char X,unsigned char Y,unsigned char data);
void delay_1us(void);
void delay_nus(unsigned int n);
void delay_1ms(void);
void delay_nms(unsigned int n);
void LCD_stuNum(void);
void LCD_Type_Vpp(char* type,float vpp);
void LCD_Freq_Vrms(uint16_t freq,float vrms);

uint8_t JudgeType(void);

void initSPI_Soft(void);
unsigned char writeByte(unsigned char data_8);
uint16_t writeWord(uint16_t data_16,uint8_t channel);
void LCD_CValue(float Cvalue);
void LCD_RValue(float Rvalue);
void suit(float RValue);

/********************************************

                                               主函数

*******************************************/

	volatile float dianzuzhi = 0;
void main()
{
	WDTCTL = WDTPW + WDTHOLD;     // 关闭看门狗
	IniClk();
	IniUART();
	IniTimerA1_ct();
	LCD_init_first();
	LCD_init();
	IniADC();
	IniButton();
	initSPI_Soft();
    __bis_SR_register(GIE);

	uint16_t data = 12;
	float voltage = 3.1;
	uint8_t cnt = 0,cnt1 = 0;
	volatile float fvpp = 0.0f;
	float floatBuf = 0;
	float floatSum = 0;
	float floatVrms = 0;
	volatile float R_know = 0.01;



	/**打印学号**/
	LCD_write_command(0x01);
	LCD_stuNum();
	LCD_stuNum();
	__delay_cycles(LONGTIME);
	measure:
	LCD_write_command(0x01);
	/**打印学号  结束**/


	next = 0;
	while(1)
	{
/****测电阻部分*****/
		/**** 控制已知电阻 ****/
		/**** initial for state 3 ***/
		P2_0_SET_HIGH;//know = 10k
		P2_1_SET_HIGH;//know = 1k
		P2_2_SET_GND;//know = 100
		/**** initial for state 3 结束***/
		switch(displayStatus)
		{
			case 0:
			{
				P2_0_SET_HIGH;//know = 10k
				P2_1_SET_HIGH;//know = 1k
				P2_2_SET_GND;//know = 100
				break;
			}
			case 1:
			{
				P2_0_SET_HIGH;//know = 10k
				P2_1_SET_GND;//know = 1k
				P2_2_SET_HIGH;//know = 100
				break;
			}
			case 2:
			{
				P2_0_SET_GND;//know = 10k
				P2_1_SET_HIGH;//know = 1k
				P2_2_SET_HIGH;//know = 100
				break;
			}
			case 3:
			{
				break;
			}
			default:
			{
				P2_0_SET_HIGH;//know = 10k
				P2_1_SET_HIGH;//know = 1k
				P2_2_SET_GND;//know = 100
				break;
			}

		};

		/**** 控制已知电阻 结束****/
		//选择已知电阻
		if(P2_0_IS_GND)
		{
			R_know = 10000.0;
		}
		else if(P2_1_IS_GND)
		{
			R_know = 1000.0;
		}
		else if(P2_2_IS_GND)
		{
			R_know = 100.0;
		}
		else
		{
			R_know = 0.01;
			PrintString("error");
		}
		//选择已知电阻 结束

		//	测量电压部分
		for(cnt = 0;cnt < 50;cnt++)
		{
			StartADCConvert();
		}

		for(cnt = 0;cnt < 50;cnt++)
		{
			floatBuf = adcbuff[cnt]* 3.55208 / 1023;
			floatSum = floatSum + floatBuf;
		}
		floatSum = floatSum / 50.0;//模拟量平均值
		PrintFloat(floatSum);
		//  测量电压部分 结束
		for(cnt = 0;cnt < 50;cnt++)
		{
			adcbuff[cnt] = 0;
		}
		//计算待测电阻值
//		dianzuzhi = (2.5 - floatSum)*R_know/floatSum ;//(2.5-V)*R_know/V公式计算电阻值
		//计算待测电阻值  结束
		if(displayStatus == 3)
		{
			suit(dianzuzhi);
		}
		//打印输出
		PrintString("the R value = ");
		PrintFloat(dianzuzhi);
		LCD_RValue(dianzuzhi);
		//打印输出 结束
		floatSum = 0;

/****测电阻部分结束*****/
/****测量电容部分*******/
    	freq = (float)(8000000.0) / totaltime;
		PrintFreq(freq);
		Cvalue = 1000000.0*1.44/360.0/freq;
		PrintString("\nCvalue = ");//乘1000000，代表电容单位是纳法拉
		PrintFloat(Cvalue);
		LCD_CValue(Cvalue);
		__delay_cycles(8000000);
/****测量电容部分*******/

//  	if(next == 1)
//  	{
//  	   goto measure;
//  	}
//  	else
//  	{
//    	// for(cnt = 0;cnt < 50;cnt ++)
//    	// {
//    	// 	writeWord(adcbuff[cnt],1);
//    	// }
//  	}
	}
}
#pragma vector = PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
	if(P1IFG & BIT3)//判断是否是P1.3产生中断
	{
		P1IFG &= ~BIT3;
//		displayStatus = (displayStatus + 1) % 4;
		if(dianzuzhi < 2000.0)
		{
			dianzuzhi = 2100.0;
		}
		else if(dianzuzhi < 20000.0)
		{
			dianzuzhi  = 21000.0;
		}
		else
		{
			dianzuzhi  = 210.0;
		}
	}
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void Time_Tick(void)
{
	static uint8_t cnt = 0;
	__bis_SR_register(GIE);//允许中断嵌套
	switch(TA1IV)
	{
	case 0x02://捕捉比较中断1
		break;
	case 0x04://捕捉比较中断2
		if(cnt == 0)
		{
			capvalue_1 = TA1CCR2;//保存第一次捕捉值
			timestamp_1 = timestamp;//保存第一次时间戳
			cnt ++;
		}
		else
		{
			capvalue_2 = TA1CCR2;//保存第二次捕捉值
			timestamp_2 = timestamp;//保存第二次时间戳
			cnt = 0;
			totaltime = (timestamp_2 - timestamp_1) * 50000 + capvalue_2 - capvalue_1;//计算总时间
		}
		break;
	case 0x0A://溢出中断
		timestamp ++;
		break;
	default:
		break;
	}
}
/*
 * 	Function declare
 */
void IniClk(void)
{
    // 时钟
    DCOCTL = CALDCO_8MHZ;
    BCSCTL1 = CALBC1_8MHZ;
    /*配置SMCLK的时钟源为DCO*/
    BCSCTL2 &= ~SELS;
    /*SMCLK的分频系数置为1*/
    BCSCTL2 &= ~(DIVS0 | DIVS1);
}

void IniButton(void)
{
    /*初始化按键所在IO口P1.3为输入*/
    P1DIR &= ~BIT3;
    /*使能P1.3口的上拉电阻*/
    P1REN |= BIT3;
    P1OUT |= BIT3;
    /*打开P1.3口中断*/
    P1IE |= BIT3;
    /*设定为下降沿触发*/
    P1IES |= BIT3;
    /*清除中断标志位*/
    P1IFG &= ~BIT3;
}
void IniADC()
{
	ADC10CTL1 |= ADC10SSEL_3;
	ADC10CTL1 |= ADC10DIV_0;
	ADC10CTL1 |= INCH_0;

	ADC10CTL0 |= SREF_0;
	ADC10CTL0 |= ADC10SHT_3;//一般不变
	ADC10CTL0 &= ~ADC10SR;

	ADC10AE0 |= BIT0;


  	  /*DTC传输模式*/
  	  ADC10DTC0 |= ADC10CT;
  	  /*传输次数*/
  	  ADC10DTC1 = 50;
  	  /*起始地址*/
  	  ADC10SA = (uint16_t)(adcbuff);


	ADC10CTL0 |= ADC10ON;
  /*允许转换*/
  ADC10CTL0 |= ENC;
}
void IniUART(void)
{
    // UART
    UCA0CTL1 |= UCSWRST;//开始初始化
    UCA0CTL0 &= ~UCSYNC;//异步模式
    UCA0CTL1 |= UCSSEL1;//时钟源 SMCLK
    	//波特率9600 8MHz
    UCA0BR0 = 65;
    UCA0BR1 = 3;
    UCA0MCTL = 2 << 1;
    //端口复用
    P1SEL |= BIT1 | BIT2;
    P1SEL2 |= BIT1 | BIT2;
    UCA0CTL1 &= ~UCSWRST;//结束初始化
    /***串口中断***/
    //    //接收中断启用
    //    IE2 |= UCA0RXIE;
    //    //清空接收中断标志
    //    IFG2 &= ~UCA0RXIFG;

}
void IniTimerA1_ct()
{
    /*设置时钟源为SMCLK*/
    TA1CTL |= TASSEL_2;
    /*设置工作模式为Up Mode*/
    TA1CTL |= MC_1;
    /*设置定时间隔*/
    TA1CCR0 = 49999;//50ms
    /*开启TAIFG中断*/
    TA1CTL |= TAIE;

    /*TA1,CCR2用于捕捉功能*/
    TA1CCTL2 |= CAP;
    /*上升沿捕捉*/
    TA1CCTL2 |= CM0;
    /*P2.5作为捕捉输入(CCI2B)*/
    TA1CCTL2 |= CCIS0;
    P2SEL |= BIT5;
    /*允许捕捉比较中断*/
    TA1CCTL2 |= CCIE;

    /*设置时钟源为SMCLK*/
    TA0CTL |= TASSEL1;
    /*设置工作模式为Up&Down*/
    TA0CTL |= MC0|MC1;
    /*设置TA0CCR0为0x00FF*/
    TA0CCR0 = 0x0AAA;
    /*设置TA0CCR1为0x00FF*/
    TA0CCR1 = 0x0555;//占空比(TACCR0 - TACCR1) / TACCR0,频率=SMCLK/(TACCR0)/2
    /*设置为比较模式*/
    TA0CCTL0 &= ~CAP;
    TA0CCTL1 &= ~CAP;
    /*设置比较输出模式*/
    TA0CCTL1 |= OUTMOD_6;
}
void SpiWriteDAC(uint16_t data,uint8_t channel)
{
	data &= ~0xf000;
	if(channel == 0)
	{
		data |= 0xC000;//高4位 1100	A channel
	}
	else
	{
		data |= 0x4000;//高4位 0100	B channel
	}
	uint8_t high = (uint8_t)(data >> 8);
	uint8_t low  = (uint8_t)(data & 0x00ff);
	P1OUT &= ~BIT5;

	SpiBusWrite(high);
	SpiBusWrite(low);

	P1OUT |= BIT5;

}
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
	LCD_write_string(0,0,"123456789");
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
void LCD_CValue(float Cvalue)
{
	const float Max10nFValue = 10.0;
	const float Max100nFValue = 100.0;
//	send Cvalue
	uint8_t charbuff_nF[] = {'#','#','#','#','.','#','#','#',' ',' ','X','1','n','F','\0'};
	uint8_t charbuff_10nF[] = {'#','#','#','#','.','#','#','#',' ',' ','X','1','0','n','F','\0'};
	uint8_t charbuff_100nF[] = {'#','#','#','#','.','#','#','#',' ',' ','X','1','0','0','n','F','\0'};

	uint16_t interger = (uint16_t)Cvalue;
	uint16_t pointNum = (uint16_t)((Cvalue - interger)*1000);
	// nF
	charbuff_nF[0] = interger / 1000 % 10 + '0';
	charbuff_nF[1] = interger / 100 % 10 + '0';
	charbuff_nF[2] = interger / 10 % 10 + '0';
	charbuff_nF[3] = interger / 1 % 10 + '0';
	/***************************************/
	charbuff_nF[5] = pointNum / 100 % 10 + '0';
	charbuff_nF[6] = pointNum / 10 % 10 + '0';
	charbuff_nF[7] = pointNum / 1 % 10 + '0';

	uint16_t interger_10 = (uint16_t)(Cvalue/10.0);
	uint16_t pointNum_10 = (uint16_t)((Cvalue/10.0 - interger_10)*1000);

	// knF
	charbuff_10nF[0] = interger_10 / 1000 % 10 + '0';
	charbuff_10nF[1] = interger_10 / 100 % 10 + '0';
	charbuff_10nF[2] = interger_10 / 10 % 10 + '0';
	charbuff_10nF[3] = interger_10 / 1 % 10 + '0';
	/***************************************/
	charbuff_10nF[5] = pointNum_10 / 100 % 10 + '0';
	charbuff_10nF[6] = pointNum_10 / 10 % 10 + '0';
	charbuff_10nF[7] = pointNum_10 / 1 % 10 + '0';

	uint16_t interger_100 = (uint16_t)(Cvalue/100.0);
	uint16_t pointNum_100 = (uint16_t)((Cvalue/100.0 - interger_100)*1000);

	// 10knF
	charbuff_100nF[0] = interger_100 / 1000 % 10 + '0';
	charbuff_100nF[1] = interger_100 / 100 % 10 + '0';
	charbuff_100nF[2] = interger_100 / 10 % 10 + '0';
	charbuff_100nF[3] = interger_100 / 1 % 10 + '0';
	/***************************************/
	charbuff_100nF[5] = pointNum_100 / 100 % 10 + '0';
	charbuff_100nF[6] = pointNum_100 / 10 % 10 + '0';
	charbuff_100nF[7] = pointNum_100 / 1 % 10 + '0';

	if(Cvalue >= Max100nFValue)
	{
		LCD_write_string(0,0,charbuff_100nF);
	}
	else if(Cvalue >= Max10nFValue)
	{
		LCD_write_string(0,0,charbuff_10nF);
	}
	else
	{
		LCD_write_string(0,0,charbuff_nF);
	}
}
void LCD_RValue(float Rvalue)
{
	const float MaxOhmValue = 2000.0;
	const float MaxKohmValue = 20000.0;
//	send Rvalue
	uint8_t charbuff_ohm[] = {'#','#','#','#','.','#','#','#',' ',' ','o','h','m','\0'};
	uint8_t charbuff_kohm[] = {'#','#','#','#','.','#','#','#',' ',' ','k','o','h','m','\0'};
	uint8_t charbuff_10kohm[] = {'#','#','#','#','.','#','#','#',' ',' ','X','1','0','k','o','h','m','\0'};

	uint16_t interger = (uint16_t)Rvalue;
	uint16_t pointNum = (uint16_t)((Rvalue - interger)*1000);
	// ohm
	charbuff_ohm[0] = interger / 1000 % 10 + '0';
	charbuff_ohm[1] = interger / 100 % 10 + '0';
	charbuff_ohm[2] = interger / 10 % 10 + '0';
	charbuff_ohm[3] = interger / 1 % 10 + '0';
	/***************************************/
	charbuff_ohm[5] = pointNum / 100 % 10 + '0';
	charbuff_ohm[6] = pointNum / 10 % 10 + '0';
	charbuff_ohm[7] = pointNum / 1 % 10 + '0';

	uint16_t interger_k = (uint16_t)(Rvalue/1000.0);
	uint16_t pointNum_k = (uint16_t)((Rvalue/1000.0 - interger_k)*1000);

	// kohm
	charbuff_kohm[0] = interger_k / 1000 % 10 + '0';
	charbuff_kohm[1] = interger_k / 100 % 10 + '0';
	charbuff_kohm[2] = interger_k / 10 % 10 + '0';
	charbuff_kohm[3] = interger_k / 1 % 10 + '0';
	/***************************************/
	charbuff_kohm[5] = pointNum_k / 100 % 10 + '0';
	charbuff_kohm[6] = pointNum_k / 10 % 10 + '0';
	charbuff_kohm[7] = pointNum_k / 1 % 10 + '0';

	uint16_t interger_10k = (uint16_t)(Rvalue/10000.0);
	uint16_t pointNum_10k = (uint16_t)((Rvalue/10000.0 - interger_10k)*1000);

	// 10kohm
	charbuff_10kohm[0] = interger_10k / 1000 % 10 + '0';
	charbuff_10kohm[1] = interger_10k / 100 % 10 + '0';
	charbuff_10kohm[2] = interger_10k / 10 % 10 + '0';
	charbuff_10kohm[3] = interger_10k / 1 % 10 + '0';
	/***************************************/
	charbuff_10kohm[5] = pointNum_10k / 100 % 10 + '0';
	charbuff_10kohm[6] = pointNum_10k / 10 % 10 + '0';
	charbuff_10kohm[7] = pointNum_10k / 1 % 10 + '0';

	switch(displayStatus)
	{
		case 0:
		{
			LCD_write_string(0,1,charbuff_ohm);
			break;
		}
		case 1:
		{
			LCD_write_string(0,1,charbuff_kohm);
			break;
		}
		case 2:
		{
			LCD_write_string(0,1,charbuff_10kohm);
			break;
		}
		case 3:
		{
			if(Rvalue >= MaxKohmValue)
			{
				LCD_write_string(0,1,charbuff_10kohm);
			}
			else if(Rvalue >= MaxOhmValue)
			{
				LCD_write_string(0,1,charbuff_kohm);
			}
			else
			{
				LCD_write_string(0,1,charbuff_ohm);
			}
			break;
		}
		default:
		{
			LCD_write_string(0,1,charbuff_kohm);
			break;
		}
	}

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
uint8_t JudgeType(void)
{
  uint8_t i = 0,j = 0;
  int16_t buf1 = 0, buf2 = 0;
  int16_t adcDivBuf = 0;
  volatile uint8_t debug = 0;
  const uint8_t charbuf1[] = {'s','q','u','a','r','e','\0'};
  const uint8_t charbuf2[] = {'t','r','i','a','n','g','l','e','\0'};
  const uint8_t charbuf3[] = {'S','i','n','\0'};
  for (i = 0; i < ADCBUFSIZE; i++)
  {
    StartADCConvert();
  }
  for (i = 0; i < ADCBUFSIZE - 1; i++)
  {
	buf1 = adcbuff[i];buf2 = adcbuff[i + 1];
	adcDivBuf = buf1 - buf2;
	typeBuff[i] = adcDivBuf;
    if(adcDivBuf < low && adcDivBuf > -low)
    {
      lowNum++;
    }
    else
    {
      highNum++;
    }
  }
  if(debug == 0)
  {debug = 1;}
  if(lowNum >= 30)
  {
    //square
	lowNum = 0; highNum = 0;
//    PrintString("square");
    return 1;
  }
  else
  {
    lowNum = 0; highNum = 0;
    for (i = 0; i < ADCBUFSIZE - 1; i++)
    {
      adcDivBuf = typeBuff[i] - typeBuff[i + 1];
      typeBuff[i] = adcDivBuf;
      if(adcDivBuf < mid && adcDivBuf > -mid)
      {
        lowNum++;
      }
      else
      {
        highNum++;
      }
    }
    if(lowNum >= 20)
    {
      //Triangle
//      PrintString("Triangle");
      return 2;

    }
    else
    {
      //Sin
//      PrintString("Sin");
      return 3;
    }
  }
}
void initSPI_Soft(void)
{
	//0--CLK 1--CS 2--DOUT 3--DIN
	P2DIR |= BIT0 + BIT1 + BIT2;
	P2DIR &= ~BIT3;
	P2OUT |= BIT0; //CLK -- 1
	P2OUT |= BIT1; //CS  -- 1
	P2OUT &= ~BIT2;//DOUT-- 0
}
unsigned char writeByte(unsigned char data_8)
{
  unsigned char cnt = 0, temp = data_8;

  for (cnt = 0; cnt < 8; cnt++)
  {
    /* spi read and write */
	P2OUT &= ~BIT0; //CLK -- 0
	if(temp & 0x80)
	{
		P2OUT |= BIT2;//DOUT -- 1
	}
	else
	{
		P2OUT &= ~BIT2;//DOUT -- 0
	}
    temp = temp << 1;
	P2OUT |= BIT0; //CLK -- 1
    temp |= (uint8_t)((P2IN | BIT3)>>3);
  }
  return temp;
}
uint16_t writeWord(uint16_t data_16,uint8_t channel)
{
  uint16_t cnt = 0, temp = data_16;
  //数据处理
  	data_16 &= ~0xf000;
	if(channel == 0x00)
	{
		data_16 |= 0xC000;//高4位 1100	A channel
	}
	else
	{
		data_16 |= 0x4000;//高4位 0100	B channel
	}
  //传输
  P2OUT &= ~BIT1; //CS  -- 0
  for (cnt = 0; cnt < 16; cnt++)
  {
    /* spi read and write */
	P2OUT &= ~BIT0; //CLK -- 0
	if(temp & 0x8000)
	{
		P2OUT |= BIT2;//DOUT -- 1
	}
	else
	{
		P2OUT &= ~BIT2;//DOUT -- 0
	}
    temp = temp << 1;
	P2OUT |= BIT0; //CLK -- 1
    temp |= (uint8_t)((P2IN | BIT3)>>3);
  }
  P2OUT |= BIT1; //CS  -- 1

  return temp;
}
/*
 * @fn:		uint16_t GetADCValue(void)
 * @brief:	进行一次ADC转换并返回ADC转换结果
 * @para:	none
 * @return:	ADC转换结果
 * @comment:ADC转换结果为10bit，以uint16_t类型返回，低10位为有效数据
 */
uint16_t GetADCValue(void)
{
	  /*开始转换*/
	  ADC10CTL0 |= ADC10SC|ENC;
	  /*等待转换完成*/
	  while(ADC10CTL1&ADC10BUSY);
	  /*返回结果*/
	  return ADC10MEM;
}
/*
 * @fn:		void StartADCConvert(void)
 * @brief:	进行一次ADC转换
 * @para:	none
 * @return:	none
 * @comment:ADC转换结果为10bit
 */
void StartADCConvert(void)
{
	  /*开始转换*/
	  ADC10CTL0 |= ADC10SC|ENC;
	  /*等待转换完成*/
	  while(ADC10CTL1&ADC10BUSY);
}
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
void suit(float RValue)
{
	const float MaxOhmValue = 2000.0;
	const float MaxKohmValue = 20000.0;

	volatile uint8_t levelSuit = 0;

	volatile uint8_t levelLowSuit = 0;
	volatile uint8_t levelMidSuit = 0;
	volatile uint8_t levelHighSuit = 0;
	levelLowSuit = RValue < MaxOhmValue && P2_2_IS_GND;
	levelMidSuit = RValue >= MaxOhmValue && RValue < MaxKohmValue && P2_1_IS_GND;
	levelHighSuit = RValue >= MaxKohmValue && P2_0_IS_GND;

	levelSuit = levelLowSuit || levelMidSuit || levelHighSuit;
	volatile uint8_t levelLess = 0;

	volatile uint8_t levelLowLess = 0;
	volatile uint8_t levelMidLess = 0;
	volatile uint8_t levelHighLess = 0;
	levelLowLess = 0;
	levelMidLess = RValue >= MaxOhmValue && RValue < MaxKohmValue && P2_2_IS_GND;
	levelHighLess = RValue >= MaxKohmValue && (!P2_0_IS_GND);

	levelLess = levelLowLess || levelMidLess || levelHighLess;

	volatile uint8_t levelHigh = 0;

	volatile uint8_t levelLowHigh = 0;
	volatile uint8_t levelMidHigh = 0;
	volatile uint8_t levelHighHigh = 0;

	levelLowHigh = RValue < MaxOhmValue && (!P2_2_IS_GND);
	levelMidHigh = RValue >= MaxOhmValue && RValue < MaxKohmValue && P2_0_IS_GND;
	levelHighHigh = 0;

	levelHigh = levelLowHigh || levelMidHigh || levelHighHigh;


	if(levelSuit)
	{
		//do nothing
	}
	else
	{
		if(levelLess)
		{
			//level up
			if(P2_2_IS_GND)
			{
				P2_0_SET_HIGH;//know = 10k
				P2_1_SET_GND;//know = 1k
				P2_2_SET_HIGH;//know = 100
			}
			else if(P2_1_IS_GND)
			{
				P2_0_SET_GND;//know = 10k
				P2_1_SET_HIGH;//know = 1k
				P2_2_SET_HIGH;//know = 100
			}
			else
			{
				//do nothing
			}
		}
		else
		{
			//level down
			if(P2_0_IS_GND)
			{
				P2_0_SET_HIGH;//know = 10k
				P2_1_SET_GND;//know = 1k
				P2_2_SET_HIGH;//know = 100
			}
			else if(P2_1_IS_GND)
			{
				P2_0_SET_HIGH;//know = 10k
				P2_1_SET_HIGH;//know = 1k
				P2_2_SET_GND;//know = 100
			}
			else
			{
				//do nothing
			}
		}
	}
}
