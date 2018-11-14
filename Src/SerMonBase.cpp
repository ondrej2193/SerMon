/*
 * SerMonBase.cpp
 *
 *  Created on: 15. 10. 2018
 *      Author: ondrej.sakala
 */

#include "SerConsts.h"
#include "SerMonBase.h"
#include "stm32f1xx_hal_uart.h"

//namespace TestNmsp {

SerMonBase::SerMonBase() {
	// TODO Auto-generated constructor stub

}

SerMonBase::~SerMonBase() {
	// TODO Auto-generated destructor stub
}

bool SerMonBase::clear(UART_HandleTypeDef *huart,const char* Arr){    /* zmazanie obrazovky */

	unsigned char  ucLocCount = 0;
    for (ucLocCount = 0; ucLocCount < MAX_ROWS; ucLocCount++) {
    	debugPrintln(huart,const_cast<char*>(Arr+(MAX_ROW_LENGTH*ucLocCount)));	// print full line
    }
    return true;
}

//} /* namespace TestNmsp */
