/*
 * SerBase.h
 *
 *  Created on: 18. 10. 2018
 *      Author: ondrej.sakala
 */

#ifndef SERBASE_H_
#define SERBASE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "SerConsts.h"
#include "stm32f1xx_hal.h"
#include <string.h>


class SerBase {
	public:  // private
		// private constructor and destructor
		SerBase();
		~SerBase();
		// private copy constructor and assignment operator
		//SerBase(const SerBase&);
		//SerBase& operator=(const SerBase&);
		//static SerBase *m_instanceSingleton;
	private:
		bool debugPrint(UART_HandleTypeDef *huart,char _out[]);
		bool debugPrintln(UART_HandleTypeDef *huart,char _out[]);
	public:
		bool clear(UART_HandleTypeDef *huart, const char* Arr);

}; // class SerBase {

#ifdef __cplusplus
}
#endif

#endif /* SERBASE_H_ */
