#ifndef LCD_H_
#define LCD_H_
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



extern void LCD_init_first(void);
extern void LCD_init(void);
extern void LCD_en_write1(void);
extern void LCD_en_write2(void);
extern void LCD_write_command(unsigned char command);
extern void LCD_write_data(unsigned char data);
extern void LCD_set_xy( unsigned char x, unsigned char y );
extern void LCD_write_string(unsigned char X,unsigned char Y,unsigned char *s);
extern void LCD_write_char(unsigned char X,unsigned char Y,unsigned char data);
extern void delay_1us(void);
extern void delay_nus(unsigned int n);
extern void delay_1ms(void);
extern void delay_nms(unsigned int n);
extern void LCD_stuNum(void);
extern void LCD_Type_Vpp(char* type,float vpp);
extern void LCD_Freq_Vrms(uint16_t freq,float vrms);

#endif /* LCD_H_ */
