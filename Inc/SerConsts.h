/*
 * SerConsts.h
 *
 *  Created on: 19. 10. 2018
 *      Author: ondrej.sakala
 */
#ifndef SERCONST_H_
#define SERCONST_H_

#include "stm32f1xx_hal.h"
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!  DEFINITIONS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
#define MAX_ROW_LENGTH_FLOAT   75.0
#define MAX_ROW_LENGTH   75
#define MAX_ROWS          5
#define MAX_BAUD_RATE   115200.0

#define MAX_TIME_DELAY_MS       (uint32_t) (((((1.0/MAX_BAUD_RATE)*10.0)*MAX_ROW_LENGTH_FLOAT)*1000.0)*2)



typedef struct {
  unsigned char       Com[5];
  bool                (*Func)(UART_HandleTypeDef *);
  unsigned int        Param;
} COM;

typedef struct {
	const char Type;
	const char *Vypis;
} PRINT_FORMAT;



#endif /* SERCONSTS_H_ */
