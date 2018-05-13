#ifndef ZXMATH_H_
#define ZXMATH_H_

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
  extern uint16_t Max(uint16_t *numptr,uint16_t num);
  extern uint16_t Min(uint16_t *numptr,uint16_t num);
  extern uint16_t Average(uint16_t *datptr);
  extern float SqrtByNewton(float x);

#endif /* ZXMATH_H_ */
