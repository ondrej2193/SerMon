/*
 * SerBase.cpp
 *
 *  Created on: 18. 10. 2018
 *      Author: ondrej.sakala
 */

#include "SerBase.h"

namespace SerBaseNmsp {

	SerBase::SerBase(void)
	{

	}

	SerBase::~SerBase(void)
	{

	}

    bool SerBase::debugPrint(UART_HandleTypeDef *huart,char _out[]) {
    	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out),1);
    	return true;
    }

    bool SerBase::debugPrintln(UART_HandleTypeDef *huart,char _out[]){
    	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out),10);
    	char newline[4] ="\r\n";
    	HAL_UART_Transmit(huart, (uint8_t *) newline,2,10);
    	return true;
    }

    bool SerBase::clear(UART_HandleTypeDef *huart,const char* Arr){    /* zmazanie obrazovky */

    	unsigned char  ucLocCount = 0;
        for (ucLocCount = 0; ucLocCount < MAX_ROWS; ucLocCount++) {
        	SerBase::debugPrintln(huart,const_cast<char*>(Arr+(MAX_ROW_LENGTH*ucLocCount)));	// print full line
        }
        return true;
    }
}

