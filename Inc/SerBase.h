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

//namespace SerBaseNmspc {

	class SerBase {
		public:  // private
			SerBase();
			~SerBase();
		public:
			bool Clear(void);
			bool WriteMem (void);
			bool ReadMem (void);
			bool SetMemAdr (void);
			bool SetMemSelector (void);
			bool TaskList (void);
			bool SetMemBase (void);
			typedef  bool (SerBase::*FredMemFn)(void);

			typedef struct {
			  unsigned char       Com[5];
			  FredMemFn           pFredMemFn;
			  unsigned int        Param;
			} COM_LINE;

			const COM_LINE ComLine[7]= {{"CLS\0",&SerBase::Clear,0},
								  {"CLS\0",&SerBase::WriteMem,0},
								  {"CLS\0",&SerBase::ReadMem,0},
								  {"CLS\0",&SerBase::SetMemAdr,0},
								  {"CLS\0",&SerBase::SetMemSelector,0},
								  {"CLS\0",&SerBase::TaskList,0},
								  {"CLS\0",&SerBase::SetMemBase,0}};
		public:
			bool SetAddress (long lAddress);
			bool SetData (long lData);
			bool GetData (long* lData);
			bool SetMemBase (long lAddress);
			bool SetMemSelector (char cMemSelect);
			bool SetHuartArr (UART_HandleTypeDef *huart, const char* pcArr);
		private:
			bool debugPrint(UART_HandleTypeDef *huart,char _out[]);
			bool debugPrintln(UART_HandleTypeDef *huart,char _out[]);
		private:
			long _lAddress;
			long _lData;
			long _lMemBase;
			char _cMemSelect;
			UART_HandleTypeDef *_huart;
			const char *_pcArr;
		public:
			char InZn;
			bool Esc,Esc2;

	}; // class SerBase {


/*	class ComTable: public SerBase {
		public:  // private
			ComTable();
			~ComTable();
		public:

			 FredMemFn pp1FredMemFn =  &SerBase::Clear;
			 FredMemFn pp2FredMemFn =  &SerBase::WriteMem;
			 FredMemFn pp3FredMemFn =  NULL;
			 char cTmpVar;
	}; // class SerBase {
*/

//} // namespace SerBaseNmspc {




#ifdef __cplusplus
}
#endif

#endif /* SERBASE_H_ */
