

#include "SerConsts.h"

namespace SerConstsNmsp {

	const char ClrScr[7]={0x1B,0x5B,0x48,0x1B,0x5B,0x4A,0x00};   /* vymazanie obrazovky */
	const char Back[4]={0x08,0x20,0x08,0x00};                   /* zmazanie znaku */
	const char CrLf[3]={0x0D,0x0A,0x00};                         /* novy riadok */

	const char* OsVerZ={" ...OS IAR PowerPac RTOS "};
	const char* VerZ={" ...program version: "};
	const char* DateZ={" ...compilation date: "};

	const char* __OS_VER__={"00.03.62"};
	const char* __PREG_VER__={"00.01.0A"};
	const char* DATE={"00.01.0A"};

	const char Hlavicka[MAX_ROWS][MAX_ROW_LENGTH]={
	  "ษอออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออป",
	  "บ         Terminal control program for Live detection organism          บ",
	  "บ                     running under FreeRTOS                            บ",
	  "บ           (C) Ondrej Sakala for (R) GlobalLogic  s.r.o.               บ",
	  "ศอออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออผ"
	};

	PRINT_FORMAT ClearPrintTab[12]={
	  {'s',ClrScr},
	  {'s',CrLf},                // novy riadok
	  {'s',OsVerZ},              // verzia OS
	  {'s',__OS_VER__},
	  {'s',CrLf},                // novy riadok
	  {'s',VerZ},                // verzia programu
	  {'s',__PREG_VER__},
	  {'s',CrLf},                // novy riadok
	  {'s',DateZ},               // datum kompilacie
	  {'s',DATE},
	  {'s',CrLf},                // novy riadok
	  {'s',CrLf}                 // novy riadok
	};

}
