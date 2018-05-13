#ifndef UARTDISPLAY_H_
#define UARTDISPLAY_H_

#include <msp430g2253.h>
#include <msp430.h>
#include "stdint.h"
/*
 * define
 */

/*
 *	Global variables
 */


 /*
  * 	Function declare
  */
  extern void UARTSendString (uint8_t str[],uint8_t len);
  extern void PrintString (uint8_t str[]);
  extern void PrintFreq(float freq);
  extern void PrintFloat(float num);
  extern void Print_Type_Real_Full(uint8_t type, float real,float full);
  
#endif /* UARTDISPLAY_H_ */
