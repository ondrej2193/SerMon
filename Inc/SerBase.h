/*
 * SerBase.h
 *
 *  Created on: 18. 10. 2018
 *      Author: ondrej.sakala
 */

#ifndef SERBASE_H_
#define SERBASE_H_

#include "SerConsts.h"
#include "stm32f1xx_hal.h"
#include <string.h>

namespace SerBaseNmsp {

class SerBase {
//public:
//	static SerBase* getInstance() {
//	    return (!m_instanceSingleton) ? m_instanceSingleton = new SerBase : m_instanceSingleton;
//	}
public:  // private
    // private constructor and destructor
	SerBase();
    ~SerBase();
    // private copy constructor and assignment operator
    SerBase(const SerBase&);
    SerBase& operator=(const SerBase&);
    static SerBase *m_instanceSingleton;
public:
    bool debugPrint(UART_HandleTypeDef *huart,char _out[]);
    bool debugPrintln(UART_HandleTypeDef *huart,char _out[]);
    bool clear(UART_HandleTypeDef *huart, const char* Arr[MAX_ROWS][MAX_ROW_LENGTH]);
public:

}; // class SerBase {

}  // namespace SerBaseNmsp {



#endif /* SERBASE_H_ */
