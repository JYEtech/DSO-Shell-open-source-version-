//////////////////////////////////////////////////////////////////////////////
//
// 	Filename:	113-15011.c
//	Version:		
//	Data:		
//
//	Author:		Liu, Zemin
//	Company:	JYE Tech
//
//-----------------------------------------------------------------------------
//
// 	Target: 		STM32F103C8 
// 	Tool chain: 	CodeSourcery G++
//
//	Descriptions: 	Main firmware for DSO Shell
//	PCB: 		109-15000-00D or later
//-----------------------------------------------------------------------------
//	Required files:
//
//-----------------------------------------------------------------------------
//	ATTENTION: 
//-----------------------------------------------------------------------------
//	Revision History:
//	
///////////////////////////////////////////////////////////////////////////////

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "Common.h"
#include "Board.h"
#include	"Screen.h"
#include	"Command.h"
#include 	"Eeprom.h"

#include	"libdso150.h"

const U8 Model[12] = {"\n\rDSO Shell"};
const U8 MFG[16] = {"\n\rJYE Tech Ltd."};
const U8 Website[20] = {"\n\rWWW.JYETECH.COM\n\r"};
const U8 FWver[20] = {"FW: 113-15011-060"};

int main (void)
{
 U16 tmp1, tmp2;
 U32 tmp;
 
 Clock_Init();
 
 Port_Init();

 /* Unlock the Flash Program Erase controller */
 FLASH_Unlock();

 /* EEPROM Init */
 EE_Init();

 USART1_Init();
 ADC2_Init();
 TFT_Init_Ili9341();
   
 uputs((U8 *)Model, USART1);
 uputs((U8 *)MFG, USART1);
 uputs((U8 *)Website, USART1);
 uputs((U8 *)FWver, USART1);

 tmp = TFT_ReadID_Ili9341();
 if((tmp & 0x0000FFFF) == 0x9341) {
	// Found 9341 controller
	TFT_Controller = 0x9341;
	}
 else {
 	TFT_Init_Ili9325();
	TFT_Controller = 0x9325;
 	}

 ClrScreen();
 SysTick_Init();
 TIM3_Init();		// Test signal
 
 NVIC_Configuration();
 
 tmp1 = clBlack;
 FillRect(ScreenX0, ScreenY0, ScreenXsize, ScreenYsize, tmp1);
 PutcGenic(24, 50, 0, clAqua, tmp1, &DSO_Shell);
 PutcGenic(120, 52, 0, clAqua, tmp1, &ByJyetech);
 PutcGenic(24, 75, 0, clAqua, tmp1, &Web);
 PutsGenic(24, 100, (U8 *)FWver, clWhite, tmp1, &ASC8X16);
 PutsGenic(24, 120, (U8 *)LibVersion, clWhite, tmp1, &ASC8X16);
  
 AppInit();  
 
 BitClr(Keypad.Flags, (1 << KF_KeyHit));
 tmp1 = 100;
 while(tmp1) {
 	KeyScan();
	Delay(65500);
	tmp1--;
 	}
 
 if(BitTest(Keypad.Flags, (1 << KF_KeyHit))) {
 	// Pause
	BitClr(Keypad.Flags, (1 << KF_KeyHit));
	while(!BitTest(Keypad.Flags, (1 << KF_KeyHit))) {
		KeyScan();
		}
 	}
 
 tmp1 = clWhite;
 FillRect(ScreenX0, ScreenY0, ScreenXsize, ScreenYsize, tmp1);

 PutcGenic(20, 71, 0, clRed, tmp1, &DSOm);
 PutcGenic(140, 70, 0, clBlue, tmp1, &Shell);
 PutcGenic(24, 115, 0, clBlack, tmp1, &Web);
 PutcGenic(20, 180, 0, clBlue, tmp1, &JYELogo);
 PutcGenic(90, 178, 0, clBlue, tmp1, &Jinyuedianzi);
 PutcGenic(90, 200, 0, clBlue, tmp1, &Jyetech);

 // LED blinks twice
 LedBlink();

 UpdateDisp(Disp_Panel | Disp_Param);
 GTimer = 1000;
 GTimeout = 0;

 // The main loop
 while(1) {
 	DsoDisplay();

	if(BitTest(AddOns, (1 << AO_MeasurementOn))) {
		OnScreenDisplay();
		}

	if(BitTest(AddOns, (1 << AO_TestSigAmpDisp))) {
		TestSigAmpDisplay();
		}
		
 	if(GTimeout) {
		GTimeout = 0;
		StartCapture();
 		}
	
	if(BitTest(Keypad.Flags, (1 << KF_DoKeyScan))) {
		BitClr(Keypad.Flags, (1 << KF_DoKeyScan));
		// Do key scan
		KeyScan();
		}

	if(Keypad.KeyCode) {
		// Process key code
		KeyProc();
		Keypad.KeyCode = 0;
		}

	tmp1 = GetDsoStatus();
	if(GTimer == 0) {
		if(BitTest(tmp1, DSO_CaptureDone)) {
			Measurements();
			UpdateDisp(Disp_Trace);
			tmp2 = GetTimebase();
			if(tmp2 <= TB_1ms) {
				// Start next capture		
				StartCapture();
				}
			else {
				// Lower capture rate
				GTimer = 40;
				}
			}
		}

	if(BitTest(tmp1, DSO_Rolling)) {
		Measurements();
		Rolling();
		}
	
 	}
 
}

