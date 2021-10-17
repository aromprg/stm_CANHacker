#ifndef PROJECT_DEF_H_
#define PROJECT_DEF_H_

#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include "stm32f10x.h"

// check if (x) power of 2 (2, 4, 8, 16, ...)
#define IS_POWER_OF_2(x) (  ( (x)&( (x)-1 )  ) == 0)

// get array items count
#define GET_ARRAY_SIZE(x) (sizeof(x)/sizeof(*(x)))

// disable/enable interrupt macros
#define DINT	__disable_irq()
#define EINT	__enable_irq()

// remap macros (in Reference manual STM32F1xxx: AFIO_MAPR SWJ_CFG[2:0] write only)
#define MAKE_REMAP(x)	AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE | (x)


#endif /* PROJECT_DEF_H_ */
