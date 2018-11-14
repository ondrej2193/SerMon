/*
 * SerMonBase.h
 *
 *  Created on: 15. 10. 2018
 *      Author: ondrej.sakala
 */

#ifndef SERMONBASE_H_
#define SERMONBASE_H_

//namespace SerMonBaseNmsp {
	class SerMonBase {
		public:
			SerMonBase();
			virtual ~SerMonBase();
			bool clear(UART_HandleTypeDef *huart,const char* Arr);
	};	// class SerMonBase
//} // namespace TestNmsp

#endif /* SERMONBASE_H_ */
