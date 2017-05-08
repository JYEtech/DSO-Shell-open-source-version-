//////////////////////////////////////////////////////////////////////////////
//
// 	Filename:	Command.c
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
//-----------------------------------------------------------------------------
//	Required files:
//
//-----------------------------------------------------------------------------
//	Notes:
//
//
//-----------------------------------------------------------------------------
//	Revision History:
//
///////////////////////////////////////////////////////////////////////////////
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "Common.h"
#include "Board.h"
#include "Command.h"
#include	"libdso150.h"
#include	"Screen.h"
#include 	"Eeprom.h"

// ===========================================================
// 	File Scope Global variables
// ===========================================================
//
const 	KeyScanCode KScanCodeTab[19] = {
//	scan code	key code		key param
	{0xFFEF, 	KC_SW1,			'1'},		// 1
	{0xFFDF, 	KC_SW2,			'2'},		// 2
	{0xFFBF, 	KC_SW3,			'3'},		// 3
	{0xFF7F, 	KC_SW4,			'4'},		// 4
	{0xFFEE, 	KC_SW1H,		'5'},		// 5
	{0xFFDE, 	KC_SW2H,		'6'},		// 6
	{0xFFBE, 	KC_SW3H,		'7'},		// 7
	{0xFF7E, 	KC_SW4H,		'8'},		// 8
	{0xFF6F, 	KC_SW1_SW4,	'A'},		// 8
	{0xFF5F, 	KC_SW2_SW4,	'B'},		// 8
	{0xFF3F, 	KC_SW3_SW4,	'C'},		// 8
	{0xFF9E, 	KC_SW2_SW3H,	'D'},		// 8
	{0xFFF7, 	KC_ENC_PB,		'P'},		// 8
	{0xFFF6, 	KC_ENC_PBH,		'Q'},		// 8
	{0xFFE7, 	KC_PB_SW1,		'a'},		// 8
	{0xFFD7, 	KC_PB_SW2,		'b'},		// 8
	{0xFFB7, 	KC_PB_SW3,		'c'},		// 8
	{0xFF77, 	KC_PB_SW4,		'd'},		// 8
	{0, 			0,			0}		
	};

const	U8	enc_table[16] = {
	0,		// 0000
	1,		// 0001
	-1,		// 0010
	0,		// 0011
	0,		// 0100
	0,		// 0101
	0,		// 0110
	0,		// 0111
	0,		// 1000
	0,		// 1001
	0,		// 1010
	0,		// 1011
	0,		// 1100
	-1,		// 1101
	1,		// 1110
	0		// 1111
	};

U16	AddOns;		// Added features

KEYPAD	Keypad;
U8	VSenPrev, CplPrev;
S16	Vmax, Vmin, Vavr, Vpp, Vrms;
float	Freq;
float	Cycle;
float	PW;
float	Duty;
U16	FreqUnit;
U16	CycleUnit;
U16	FreqStrLen = 0;
U16	CycleStrLen = 0;
U16	PwStrLen = 0;

U8	ReadingStr[22];
U16	Flags;

const	float	UnitRate[12] = {
	1000.0,		// 	VS_20V,
	500.0,		// 	VS_10V,
	250.0,		// 	VS_5V,
	100.0,		// 	VS_2V,
	50.0,		// 	VS_1V,
	25.0,		// 	VS_05V,
	10.0,		// 	VS_02V,
	5.0,			// 	VS_01V,
	2.5,			// 	VS_50mV,
	1.0,			// 	VS_20mV,
	0.5,			// 	VS_10mV,	
	0.25			//	VS_5mV
	};

const float	SampleRate[TBMax - TBMin + 1] = {
	(25.0/500.0),					//	TB_500s,		//			20s
	(25.0/200.0),					//	TB_200s,		//			8s
	(25.0/100.0),					//	TB_100s,		//			4s
	(25.0/50.0),					//	TB_50s,		//			2s
	(25.0/20.0),					//	TB_20s,		// 1.25Hz	800ms
	(25.0/10.0),					//	TB_10s,		// 2.5Hz		400ms
	(25.0/5.0),					//	TB_5s,		// 5Hz		200ms
	(25.0/2.0),					//	TB_2s,		// 12.5Hz	80ms
	(25.0/1.0),					//	TB_1s,		// 25Hz		40ms
	(25.0/0.5),					//	TB_05s,		// 50Hz		20ms
	(25.0/0.2),					//	TB_02s,		// 125Hz		8ms
	(25.0/0.1),					//	TB_01s,		// 250Hz		4ms
	(25.0/50.0) * 1000.0,			//	TB_50ms,	// 500Hz		2ms
	(25.0/20.0) * 1000.0,			//	TB_20ms,	// 1.25K	800us
	(25.0/10.0) * 1000.0,			//	TB_10ms,	// 2.5K	400us
	(25.0/5.0) * 1000.0,			//	TB_5ms,		// 5K	200us
	(25.0/2.0) * 1000.0,			//	TB_2ms,		// 12.5K	80us
	(25.0/1.0) * 1000.0,			//	TB_1ms,		// 25K	40us
	(25.0/0.5) * 1000.0,			//	TB_05ms,	// 50K	20us
	(25.0/0.2) * 1000.0,			// 	TB_02ms,	// 125K	8us
	(25.0/0.1) * 1000.0,			// 	TB_01ms,	// 250K	4us 
	(25.0/50.0) * 1000000.0,			// 	TB_50us,		// 500K	2us 
	(25.0/20.0) * 1000000.0,			// 	TB_20us,		// 1.25M	0.8us **** used 1M instead. Interpolation is required to make 1.25M
	(25.0/10.0) * 1000000.0,			//	TB_10us,		// 2.5M	0.4us **** use Dual ADC mode to achieve 2M. Interpolation for 2.5M
//	16			//	TB_5us,		// 5M	0.2us 
//	200,			//	TB_2us,		// 
//	200,			//	TB_1us,
//	200			//	TB_05us,
//	200			// 	TB_max
	};

U8	*UnitsStr[8] = {
	(U8 *)"Hz",
	(U8 *)"KHz",
	(U8 *)"MHz",
	(U8 *)"GHz",
	(U8 *)"s",
	(U8 *)"ms",
	(U8 *)"us",
	(U8 *)"ns"
};

// ===========================================================
//	Function Definitions
// ===========================================================
//
void	AppInit()
{
 U16 tmp0;

// =============================

 // Check EEPROM for valid settings
 EE_ReadVariable(Addr_SettingStatus, &tmp0);
 if(tmp0 == SettingStatus_Initialized) {
	// Load saved settings
	EE_ReadVariable(VirtAddVarTab[Addr_Vpos], &tmp0);
	SetVPos(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_Vsen], &tmp0);
	SetVSen(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_Cpl], &tmp0);
	SetCpl(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_TimeBase], &tmp0);
	SetTimeBase(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_TrigMode], &tmp0);
	SetTrigMode(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_TrigEdge], &tmp0);
	SetTrigEdge(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_TrigLvl], &tmp0);
	SetTrigLvl(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_RecLen], &tmp0);
	SetRecLen(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_HPos], &tmp0);
	SetHPos(tmp0);
//	EE_ReadVariable(VirtAddVarTab[Addr_VPosOfs], &tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_VPosOfs00 + (GetVSen() - VSenMin)], &tmp0);

	SetVPosOfs(tmp0);
	EE_ReadVariable(VirtAddVarTab[Addr_AddOns], &AddOns);
	}
 else {
	// Load default settings and initialize EEPROM
	LoadDefault();
	
	// Mark down EEPROM has been initialized
	EE_WriteVariable(VirtAddVarTab[Addr_SettingStatus], SettingStatus_Initialized);

 	}
 SetTrigPos(GetRecLen()/2);
 SetTrigSen(10);
 
 OutputVSen();

// =============================
// Note: DSO_Init() must be executed for proper running of the capture engine
 
 DSO_Init();

 // Misc initialization
 TimerKeyScan = 1;

 Keypad.KDebounceVal = KD_val;

 FreqStrLen = 0;
 CycleStrLen = 0;
 PwStrLen = 0;

 Flags = 0;
}

void	LoadDefault(void)
{
 U8 tmp0;
 
 SetVPos(0);
 EE_WriteVariable(VirtAddVarTab[Addr_Vpos], 0);
 SetVSen(VS_05V);
 EE_WriteVariable(VirtAddVarTab[Addr_Vsen], VS_05V);
 SetCpl(CP_DC);
 EE_WriteVariable(VirtAddVarTab[Addr_Cpl], CP_DC);
 SetTimeBase(TB_1ms);
 EE_WriteVariable(VirtAddVarTab[Addr_TimeBase], TB_1ms);
 SetTrigMode(TM_Auto);
 EE_WriteVariable(VirtAddVarTab[Addr_TrigMode], TM_Auto);
 SetTrigEdge(TE_Falling);
 EE_WriteVariable(VirtAddVarTab[Addr_TrigEdge], TE_Falling);
 SetTrigLvl(0);
 EE_WriteVariable(VirtAddVarTab[Addr_TrigLvl], 0);
 SetRecLen(SampleBufSizeMax);
 EE_WriteVariable(VirtAddVarTab[Addr_RecLen], SampleBufSizeMax);
 SetHPos(GetRecLen()/2 - WDsize/2);
 EE_WriteVariable(VirtAddVarTab[Addr_HPos], GetRecLen()/2 - WDsize/2);
 SetVPosOfs(0);
 EE_WriteVariable(VirtAddVarTab[Addr_VPosOfs], 0);
	
 EE_WriteVariable(VirtAddVarTab[Addr_AddOns], 0);

 tmp0 = VSenMin;
 while(tmp0 <= VSenMax) {
	EE_WriteVariable(VirtAddVarTab[Addr_VPosOfs00 + (tmp0 - VSenMin)], 0);
	tmp0++;
 	}
 
}

void	KeyProc(void)
{
 switch(Keypad.KeyCode) {
	case KC_SW1:
		SelVertical();
	default:	
		break;
		
	case KC_SW2:
		SelHorzontal();
		break;
		
	case KC_SW3:
		SelTrigger();
		break;

	case KC_ENC_CW:
		DoKeyInc();
		break;

	case KC_ENC_CCW:
		DoKeyDec();
		break;

	case KC_SW4:
		DoKeyOk();
		break;

	case KC_SW1H:
		VPosAlign();
		break;

	case KC_SW2H:
		CenterHPos();
		break;

	case KC_SW3H:
		CenterTrigLevel();
		break;
		
	case KC_SW4H:
		DoKeyOkH();
		break;

	case KC_PB_SW2:
		SaveWaveform();
		break;

	case KC_PB_SW3:
		LoadWaveform();
		break;

	case KC_SW2_SW3H:
		LoadDefault();
		GTimeout = 1;

	case KC_ENC_PB:
		if(BitTest(AddOns, (1 << AO_TestSigAmpDisp))) {
			// Toggle amp
			BitXor(AddOns, (1 << AO_TestSigAmp));
			if(BitTest(AddOns, (1 << AO_TestSigAmp))) {
				// Set to 0.1V
				GPIOB->CRH = ((GPIO_CNF_AF_PP | GPIO_Mode_Out50M) << ((8 - 8)*4))		// Output, Trigger level
				 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << ((9 - 8)*4))		// Output, TFT_nRESET
				 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << ((10 - 8)*4))		// SCL
				 			|((GPIO_CNF_Floating | GPIO_Mode_In) << ((11 - 8)*4))		// SDA
				 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << ((12 - 8)*4))		// AMPSEL (for test signal)
				 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((13 - 8)*4))			// 
				 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((14 - 8)*4))			// 
				 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((15 - 8)*4));			// 
				// GPIOB->ODR &= ~(1 << 12);			
 				Port_BitClr(GPIOB, (1 << 12));
				}
			else {
				// Set to 3.3V
				GPIOB->CRH = ((GPIO_CNF_AF_PP | GPIO_Mode_Out50M) << ((8 - 8)*4))		// Output, Trigger level
				 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << ((9 - 8)*4))		// Output, TFT_nRESET
				 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << ((10 - 8)*4))		// SCL
				 			|((GPIO_CNF_Floating | GPIO_Mode_In) << ((11 - 8)*4))		// SDA
				 			|((GPIO_CNF_Floating | GPIO_Mode_In) << ((12 - 8)*4))		// AMPSEL (for test signal)
				 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((13 - 8)*4))			// 
				 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((14 - 8)*4))			// 
				 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((15 - 8)*4));			// 
				}
			}
		break;

	case KC_ENC_PBH:
		BitXor(AddOns, (1 << AO_TestSigAmpDisp));
		if(!BitTest(AddOns, (1 << AO_TestSigAmpDisp))) {
			// Clear amp display
			FillRect(TestSigAmpx0, TestSigAmpy0, 14 * 8, 16, clBlack);
			}
		break;
 	}
}


void KeyScan(void)
{
 U16	tmp1, tmp4;
 U8	tmp2, tmp3;

 Keypad.KScanBuf = NoKey;
 // Set keypad ports to input with pull-ups
 GPIOB->CRL = ((GPIO_CNF_IPU | GPIO_Mode_In) << (0*4))				// TFT port - D0, ENC_A
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << (1*4))				// TFT port - D1, ENC_B
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << (2*4))				// TFT port - D2
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << (3*4))				// TFT port - D3, ENC_PB
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << (4*4))				// TFT port - D4, SW2
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << (5*4))				// TFT port - D5, SW3
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << (6*4))				// TFT port - D6, SW4	
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << (7*4));				// TFT port - D7, SW5		
 Delay(10);
 
 // Read buttons
 tmp1 = (PB_Port & PB_Bits) | ~PB_Bits;
 tmp4 = ENC_Port & ENC_Bits;
 
 // Restore
 GPIOB->CRL = ((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (0*4))		// TFT port - D0, ENC_A
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (1*4))		// TFT port - D1, ENC_B
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (2*4))		// TFT port - D2
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (3*4))		// TFT port - D3, ENC_PB
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (4*4))		// TFT port - D4, SW2
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (5*4))		// TFT port - D5, SW3
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (6*4))		// TFT port - D6, SW4	
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (7*4));		// TFT port - D7, SW5		

 if(tmp1 != NoKey) {
	BitSet(Keypad.Flags, (1 << KF_KeyHit));
	Keypad.KScanBuf = tmp1;
 	}

 // -- Debouncing
 if((Keypad.KScanBuf == NoKey) || (Keypad.KScanBuf != Keypad.KScanCode)) {
 	if(BitSet(Keypad.Flags, (1 << KF_PressDetected))) {
		KeyConvert((KeyScanCode *)KScanCodeTab, Keypad.KScanCode);
		BitClr(Keypad.Flags, (1 << KF_PressDetected));
 		}
 	Keypad.KScanCode = Keypad.KScanBuf;
	Keypad.KCount = 0;
	Keypad.KHCount = 0;
	Keypad.KTimeChk = KH_val;
	BitClr(Keypad.Flags, (1 << KF_HoldDetected));
 	}
 else {
	Keypad.KCount++;
	if(Keypad.KCount > Keypad.KDebounceVal) {
		if(Keypad.KCount == Keypad.KDebounceVal + 3) {
			Keypad.KCount = Keypad.KDebounceVal;
			if(++Keypad.KHCount == Keypad.KTimeChk) {
				// Key hold detected
				BitClr(Keypad.Flags, (1 << KF_PressDetected));
				if(!BitTest(Keypad.Flags, (1 << KF_HoldDetected))) {
					KeyConvert((KeyScanCode *)KScanCodeTab, Keypad.KScanCode & 0xFFFE);
					BitSet(Keypad.Flags, (1 << KF_HoldDetected));
					Keypad.KTimeChk += 20;
					}
				else {
					KeyConvert((KeyScanCode *)KScanCodeTab, Keypad.KScanCode);
					// Change KTimeChk for key repeat
					Keypad.KTimeChk += KR_Time;
					}
				}
			}
		}
	else if(Keypad.KCount == Keypad.KDebounceVal) {
		// Key push detected
		BitSet(Keypad.Flags, (1 << KF_PressDetected));
		}
	}

 // Process encoder
 Keypad.EncState <<= 2;
 Keypad.EncState |= tmp4;
 tmp1 = enc_table[Keypad.EncState & 0x000F];
 if(tmp1 == 1) {
	Keypad.KeyCode = KC_ENC_CCW;
	Keypad.KeyParam = 0;
	}
 if(tmp1 == 0xFF) {
	Keypad.KeyCode = KC_ENC_CW;
	Keypad.KeyParam = 1;
	}
 
#define	Threshold_High		0x0900
#define	Threshold_Low		0x0300
 
 // Read switch Cpl
 tmp1 = ADC_Poll(ADC2, 5);
 tmp2 = 0;
 if(tmp1 > Threshold_High) {
 	tmp2 = 1;
 	}
 else if(tmp1 < Threshold_Low) {
 	tmp2 = 2;
 	}
 
 // Determine Cpl setting
 if(tmp2 != CplPrev) {
	SetCpl(tmp2);
	CplPrev = tmp2;
 	UpdateDisp(Disp_Param);
 	}
 }

void	KeyConvert(KeyScanCode *KSCTab, U16 KSCode)
{
 U16 tmp1;
 
 while((tmp1 = *(U16 *)(KSCTab + 0))) {
	if(tmp1 == KSCode) {
		// -- Match found
		Keypad.KeyCode = *(U8 *)((U8 *)KSCTab + 2);
		Keypad.KeyCodeBuf = Keypad.KeyCode;
		Keypad.KeyParam = *(U8 *)((U8 *)KSCTab + 3);
		return;
		}
	else {
		// -- Proceed to next entry
		KSCTab = (KeyScanCode *)((U8 *)KSCTab + sizeof(KeyScanCode));
		}
	
 	}
}

void	DoKeyOk(void)
{
 U8 tmp;
 U8 tmpbuf[33];
 U32	tmp1;

 tmp = GetDsoStatus();
 // Toggle HOLD state
 BitXor(tmp, DSO_Hold);

 if(BitTest(tmp, DSO_Hold)) {
 	// Set HOLD 
 	SetHold();
 	// Stop capture
 	StopCapture();
 	}
 else {
 	// Clear HOLD
 	ClrHold();
 	// Start capture at exit of HOLD
	StartCapture();
 	}
 
 UpdateDisp(Disp_Param);
}

void	DoKeyInc(void)
{
 S8	tmp0;
 S16	tmp1;

 switch(GetFocus()) {
	case FC_VSen:
 		tmp0 = GetVSen();
 		tmp0++;
 		tmp0 = SetVSen(tmp0);
		OutputVSen();
		EE_WriteVariable(VirtAddVarTab[Addr_Vsen], tmp0);
		// Update VPos offset
		EE_ReadVariable(VirtAddVarTab[Addr_VPosOfs00 + (tmp0 - VSenMin)], &tmp1);
		SetVPosOfs(tmp1);
		break;

	case FC_Timebase:
 		tmp0 = GetTimebase();
 		tmp0++;
 		tmp0 = SetTimeBase(tmp0);
		EE_WriteVariable(VirtAddVarTab[Addr_TimeBase], tmp0);
		if(tmp0 >= TB_20ms) {
			// Restart capture
			StartCapture();
			}
		else {
			// Change sampling rate only
			UpdateTimebase();
			}
		// Make key debounce time shorter for these TB's 
		if((tmp0 < TB_20ms) && (tmp0 > TB_1s)) {
			Keypad.KDebounceVal = KD_val1;
			}
		else {
			Keypad.KDebounceVal = KD_val;
			}
	default:	
		break;
		
	case FC_TrigMode:
 		tmp0 = GetTrigMode();
 		tmp0++;
 		tmp0 = SetTrigMode(tmp0);
		EE_WriteVariable(VirtAddVarTab[Addr_TrigMode], tmp0);
		// Restart capture. 
		StartCapture();
		break;

	case FC_TrigEdge:
 		tmp0 = GetTrigEdge();
 		tmp0++;
 		tmp0 = SetTrigEdge(tmp0);
		EE_WriteVariable(VirtAddVarTab[Addr_TrigEdge], tmp0);
		break;
		
	case FC_VPos:
 		tmp1 = GetVPos();
 		tmp1++;
 		tmp1 = SetVPos(tmp1);
		EE_WriteVariable(VirtAddVarTab[Addr_Vpos], tmp1);
 		UpdateDisp(Disp_Trace);
		break;

	case FC_TrigLvl:
 		tmp1 = GetTrigLvl();
 		tmp1 += 2;
 		tmp1 = SetTrigLvl(tmp1);
		EE_WriteVariable(VirtAddVarTab[Addr_TrigLvl], tmp1);
//		OutputTLvl();
		break;
		
	case FC_HPos:
		// Move waveform right
 		tmp1 = GetHPos();
 		tmp1--;
 		tmp1 = SetHPos(tmp1);
		EE_WriteVariable(VirtAddVarTab[Addr_HPos], tmp1);
 		UpdateDisp(Disp_Trace);
		break;
		
 	}
 
 UpdateDisp(Disp_Param);
}

void	DoKeyDec(void)
{
 S8	tmp0;
 S16	tmp1;
 
 switch(GetFocus()) {
	case FC_VSen:
 		tmp0 = GetVSen();
 		tmp0--;
 		tmp0 = SetVSen(tmp0);
		OutputVSen();
		EE_WriteVariable(VirtAddVarTab[Addr_Vsen], tmp0);
		// Update VPos offset
		EE_ReadVariable(VirtAddVarTab[Addr_VPosOfs00 + (tmp0 - VSenMin)], &tmp1);
		SetVPosOfs(tmp1);
		break;

	case FC_Timebase:
 		tmp0 = GetTimebase();
 		tmp0--;
 		tmp0 = SetTimeBase(tmp0);
		EE_WriteVariable(VirtAddVarTab[Addr_TimeBase], tmp0);
		if(tmp0 >= TB_50ms) {
			// Restart capture
			StartCapture();
			}
		else {
			// Change sampling rate only
			UpdateTimebase();
			
			}

	default:	
		break;
		
	case FC_TrigMode:
 		tmp0 = GetTrigMode();
 		tmp0--;
 		tmp0 = SetTrigMode(tmp0);
		EE_WriteVariable(VirtAddVarTab[Addr_TrigMode], tmp0);
		// Restart capture. 
		StartCapture();
		break;

	case FC_TrigEdge:
 		tmp0 = GetTrigEdge();
 		tmp0--;
 		tmp0 = SetTrigEdge(tmp0);
		EE_WriteVariable(VirtAddVarTab[Addr_TrigEdge], tmp0);
		break;
		
	case FC_VPos:
 		tmp1 = GetVPos();
 		tmp1--;
 		tmp1 = SetVPos(tmp1);
		EE_WriteVariable(VirtAddVarTab[Addr_Vpos], tmp1);
 		UpdateDisp(Disp_Trace);
		break;

	case FC_TrigLvl:
 		tmp1 = GetTrigLvl();
 		tmp1 -= 2;
 		tmp1 = SetTrigLvl(tmp1);
		EE_WriteVariable(VirtAddVarTab[Addr_TrigLvl], tmp1);
//		OutputTLvl();
		break;
		
	case FC_HPos:
		// Move waveform left
 		tmp1 = GetHPos();
 		tmp1++;
 		tmp1 = SetHPos(tmp1);
		EE_WriteVariable(VirtAddVarTab[Addr_HPos], tmp1);
 		UpdateDisp(Disp_Trace);
		break;
		
 	}
 
 UpdateDisp(Disp_Param);
}

void	DoKeyOkH(void)
{
 
 BitXor(AddOns, (1 << AO_MeasurementOn));
 EE_WriteVariable(VirtAddVarTab[Addr_AddOns], AddOns);
 if(!BitTest(AddOns, (1 << AO_MeasurementOn))) {
	// Clear 
	FillRect(VoltageReadingx0, VoltageReadingy0, 100, 80, clBlack);
	FillRect(FreqReadingx0, FreqReadingy0, 140, 70, clBlack);
	}
}

void	DoKeyIncH(void)
{
 S16	tmp1;
 
 switch(GetFocus()) {
	case FC_VPos:
 		tmp1 = GetVPos();
 		tmp1 += 10;
 		SetVPos(tmp1);
		break;

	case FC_TrigLvl:
 		tmp1 = GetTrigLvl();
 		tmp1 += 40;
 		SetTrigLvl(tmp1);
		break;
		
	case FC_HPos:
		// Move waveform right
 		tmp1 = GetHPos();
 		tmp1 -= 20;
 		SetHPos(tmp1);		
		break;
 	}

 UpdateDisp(Disp_Param);
}

void	DoKeyDecH(void)
{
 S16	tmp1;
 
 switch(GetFocus()) {
	case FC_VPos:
 		tmp1 = GetVPos();
 		tmp1 -= 10;
 		SetVPos(tmp1);
		break;

	case FC_TrigLvl:
 		tmp1 = GetTrigLvl();
 		tmp1 -= 40;
 		SetTrigLvl(tmp1);
		break;
		
	case FC_HPos:
		// Move waveform left
 		tmp1 = GetHPos();
 		tmp1 += 20;
 		SetHPos(tmp1);
		break;
 	}

 UpdateDisp(Disp_Param);
}

void	DoKeySelH(void)
{
}

void	SelVertical(void)
{
 U8 tmp;

 tmp = GetFocus();
 if(tmp != FC_VSen) {
 	tmp = FC_VSen;
 	}
 else {
	tmp = FC_VPos;
 	}

 SetFocus(tmp);
 UpdateDisp(Disp_Param);
}

void SelHorzontal(void)
{
 U8 tmp;

 tmp = GetFocus();
 if(tmp != FC_Timebase) {
 	tmp = FC_Timebase;
 	}
 else {
	tmp = FC_HPos;
 	}

 SetFocus(tmp);
 UpdateDisp(Disp_Param);
}

void	SelTrigger(void)
{
 U8 tmp;

 tmp = GetFocus();
 switch(tmp) {
	case FC_TrigMode:
		tmp = FC_TrigLvl;
		break;
		
	case FC_TrigEdge:
		tmp = FC_TrigMode;
		break;

	case FC_TrigLvl:
		tmp = FC_TrigEdge;
		break;

	default:
		tmp = FC_TrigMode;
 	}
 
 SetFocus(tmp);
 UpdateDisp(Disp_Param);
}

void	VPosAlign(void)
{
 S16 tmp1;
 S8	tmp2, tmp3, tmp4, tmp5;
 
 // Do VPos alignment
		
 // Set timebase to 1ms so as the calibration won't take too long
 // Set trigger mode to AUTO
 tmp3 = GetTimebase();
 tmp4 = GetVSen();
 tmp5 = GetTrigMode();
		
 SetTimeBase(TB_1ms);	
 SetTrigMode(TM_Auto);

 // Make sure the current VPosOfs is zero
 SetVPosOfs(0);
 
 // Do alignment for each VSen setting
 tmp2 = VSenMin;
 while(tmp2 <= VSenMax) {
	SetVSen(tmp2);
	OutputVSen();
	Delay(60000);
	StartCapture();
	tmp1 = 0;
	while(!BitTest(tmp1, DSO_CaptureDone)) {
		tmp1 = GetDsoStatus();
		}
	tmp1 = (S16)(GetAverage() - SampleMidValue);
	EE_WriteVariable(VirtAddVarTab[Addr_VPosOfs00 + (tmp2 - VSenMin)], tmp1);
	tmp2++;
	}
		
 // Restore settings
 SetTimeBase(tmp3);
 SetVSen(tmp4);
 SetTrigMode(tmp5);
 OutputVSen();
 // Setup current VPos offset
 EE_ReadVariable(VirtAddVarTab[Addr_VPosOfs00 + (tmp4 - VSenMin)], &tmp1);
 SetVPosOfs(tmp1);

}

void	CenterHPos(void)
{
 // Set HPos to center
 SetHPos((GetRecLen() - WWindowSizex)/2);
 UpdateDisp(Disp_Param);
}

void	CenterTrigLevel(void)
{
 S16 tmp1;
 
 // Set trigger level to the middle of signal amplitude
 tmp1 = ((Vmax + Vmin)/2) + 2 * GetVPosOfs();	
 SetTrigLvl(tmp1);
 UpdateDisp(Disp_Param);
}

void	LedBlink(void)
{
 U16	tmp;
 
 // Turn on LED
 Port_BitClr(LED_Base, (1 << LED_Bit));
 tmp = 50;
 while(tmp) {
 	Delay(65000);
 	tmp--;
 	}
 
 // Turn off LED
 Port_BitSet(LED_Base, (1 << LED_Bit));
 tmp = 50;
 while(tmp) {
 	Delay(65000);
 	tmp--;
 	}

 // Turn on LED
 Port_BitClr(LED_Base, (1 << LED_Bit));
 tmp = 50;
 while(tmp) {
 	Delay(65000);
 	tmp--;
 	}
 
 // Turn off LED
 Port_BitSet(LED_Base, (1 << LED_Bit));
}

void	LedBlink_Fast(void)
{
 U16 tmp;

 while(1) {
 	// Turn on LED
 	Port_BitClr(LED_Base, (1 << LED_Bit));
 	tmp = 1;
 	while(tmp) {
 		Delay(65000);
 		tmp--;
 		}
 
 	// Turn off LED
 	Port_BitSet(LED_Base, (1 << LED_Bit));
 	tmp = 1;
 	while(tmp) {
 		Delay(65000);
 		tmp--;
 		}
 	}
}

void	SaveWaveform(void)
{
 U16	tmp0, tmp1, tmp2;
 U16	*ptmp1;
 
 // Display message
 PutsGenic(WWindowx0 + 4, WWindowy0 -17, (U8 *)"Saving ", clOrange, clBlack, &ASC8X16);

 FLASH_Unlock();
  
 // Erase buffer
 tmp2 = FLASH_ErasePage(Waveform_START_ADDRESS);
 if(tmp2 != FLASH_COMPLETE) {
	// Failed
	PutsGenic(WWindowx0 + 4, WWindowy0 -17, (U8 *)"Failed1", clRed, clBlack, &ASC8X16);
	goto Failure;
 	}
 tmp2 = FLASH_ErasePage(Waveform_START_ADDRESS + 0x0400);
 if(tmp2 != FLASH_COMPLETE) {
	// Failed
	PutsGenic(WWindowx0 + 4, WWindowy0 -17, (U8 *)"Failed1", clRed, clBlack, &ASC8X16);
	goto Failure;
 	}
 tmp2 = FLASH_ErasePage(Parameter_START_ADDRESS);
 if(tmp2 != FLASH_COMPLETE) {
	// Failed
	PutsGenic(WWindowx0 + 4, WWindowy0 -17, (U8 *)"Failed2", clRed, clBlack, &ASC8X16);
	goto Failure;
 	}

 // Save waveform data
 ptmp1 = SampleBuf;
 tmp1 = GetRecLen();
 tmp0 = 0;
 while(tmp0 < tmp1) {
 	tmp2 = FLASH_ProgramHalfWord(Waveform_START_ADDRESS + tmp0 * 2, *ptmp1);
	if(tmp2 != FLASH_COMPLETE) {
		PutsGenic(WWindowx0 + 4, WWindowy0 -17, (U8 *)"Failed3", clRed, clBlack, &ASC8X16);
		goto Failure;
		}
	// Proceed to next sample
	ptmp1++;
	tmp0++;
 	}

 // Save parameters
 ptmp1 = ReadingStr;
 *ptmp1++ = (U16)GetRecLen();
 *ptmp1++ = (U16)GetVSen();
 *ptmp1++ = (U16)GetCpl();
 *ptmp1++ = (U16)GetTimebase();
 *ptmp1++ = (U16)GetTrigMode();
 *ptmp1++ = (U16)GetTrigEdge();
 *ptmp1++ = (U16)GetTrigLvl();
 *ptmp1++ = (U16)GetHPos();
 *ptmp1++ = (U16)GetVPos();
 *ptmp1++ = (U16)GetVPosOfs();
 
 ptmp1 = ReadingStr;
 tmp1 = 10;
 tmp0 = 0;
 while(tmp0 < tmp1) {
 	tmp2 = FLASH_ProgramHalfWord(Parameter_START_ADDRESS + tmp0 * 2, *ptmp1);
	if(tmp2 != FLASH_COMPLETE) {
		PutsGenic(WWindowx0 + 4, WWindowy0 -17, (U8 *)"Failed4", clRed, clBlack, &ASC8X16);
		goto Failure;
		}
	// Proceed to next sample
	ptmp1++;
	tmp0++;
 	}

 // Return to normal running
 UpdateDisp(Disp_Param);
 return;

 Failure:
	UartPutc(0xFA, USART1);
	UartPutc(tmp2 >> 8, USART1);
	UartPutc(tmp2, USART1);
	UartPutc(0xF0, USART1);
	UartPutc(tmp0 >> 8, USART1);
	UartPutc(tmp0, USART1);
	
 	while(1);
}

void	LoadWaveform(void)
{
 U16	tmp0, tmp1;
 U16	*ptmp1, *ptmp2;

 // Display message
 PutsGenic(WWindowx0 + 4, WWindowy0 -17, (U8 *)"Loading", clOrange, clBlack, &ASC8X16);

 // Load parameters
 ptmp2 = Parameter_START_ADDRESS;
 SetRecLen(*ptmp2++);
 SetVSen((S8)*ptmp2++);
 SetCpl((S8)*ptmp2++);
 SetTimeBase((S8)*ptmp2++);
 SetTrigMode((S8)*ptmp2++);
 SetTrigEdge((S8)*ptmp2++);
 SetTrigLvl((S16)*ptmp2++);
 SetHPos((S16)*ptmp2++);
 SetVPos((S16)*ptmp2++);
 SetVPosOfs((S16)*ptmp2++);
 
 // Load waveform data to SamBuffer
 ptmp1 = SampleBuf;
 ptmp2 = (U16 *)Waveform_START_ADDRESS;
 tmp1 = GetRecLen();
 tmp0 = 0;
 while(tmp0 < tmp1) {
 	*ptmp1++ = *ptmp2++;
	tmp0++;
 	}

 // Enter HOLD
 SetHold();
 // Stop capture
 StopCapture();

 // Display
 UpdateDisp(Disp_Param);
 UpdateDisp(Disp_Trace);
 
}

// ---------------------------------------------------------------
// Note: Freq and duty measurement produce large errors for timebase >= 20us.
//	This is because interpolation has been used for these timebases.

#define	LevelSen		16		// Sensitivity of level

void	Measurements(void)
{
 S32 sum, bufsize, index, rms;
 S16 vmax, vmin, v0;
 S16 tmp;
 S16 currentstate;
 S32	fallingcnt, risingcnt;
 S32	fallingtime, risingtime, highleveltime;
 S16 HighLevel, LowLevel;
 S32 firstfalling, firstrising;	// Keep the start of valid samples
 
 // Initialization for voltage measurement
 vmax = 0;
 vmin = 0xFFF;
 sum = 0;
 rms = 0;
 v0 = GetVPosOfs();	// 
 
 bufsize = GetRecLen();
 index = 0;
 while(index < bufsize) {
 	// Get a sample
	tmp = *(SampleBuf + index);

	// Searching for Vmax
	if(tmp > vmax) {
		vmax = tmp;
		}

	// Searching for Vmin
	if(tmp < vmin) {
		vmin = tmp;
		}

	// Calculate sum 
	sum += tmp;

	// Calculate rms
	rms += ((S32)(tmp - SampleMidValue) * (S32)(tmp - SampleMidValue));

	index++;
 	}

 BitSet(Flags, (1 << Flag_VoltValid));
 if((vmax > 0x0FFC) || (vmin < 0x0004)) {
 	BitClr(Flags, (1 << Flag_VoltValid));
 	}
 	
 Vmax = vmax - SampleMidValue;
 Vmin = vmin - SampleMidValue;
 Vavr = (sum/bufsize) - SampleMidValue;
 Vpp = vmax - vmin;
 Vrms = (S16)(sqrt((float)rms/(float)bufsize));

 // Initialization for freq, Cycle, and duty measurement
 v0 = (sum/bufsize);
 fallingcnt = 0;	
 risingcnt = 0;
 HighLevel = v0 + LevelSen/2;
 LowLevel = v0 - LevelSen/2;
 v0 = GetVPosOfs();	// 
 currentstate = ((*SampleBuf - v0) >= SampleMidValue) ? 1 : 0;
 highleveltime = 0;

 index = 0;
 while(index < bufsize) {
 	// Get a sample
	tmp = *(SampleBuf + index) - v0;

	// Processing cycle by cycle
	if(currentstate == 1) {
		// Currrent level is HIGH
		// Searching for falling transition
		if(tmp <= LowLevel) {
			// Falling transition found
			currentstate = 0;
			fallingcnt++;
			fallingtime = index;		// Mark down the falling edge 
			if(fallingcnt == 1) {
				// Mark down the start of valid sample
				firstfalling = index;
				}
			if(risingcnt != 0) {
				// Accumulate high level duration
				highleveltime += (fallingtime - risingtime);
				}
			}
		}
	else {
		// Currrent level is LOW
		// Searching for rising transition
		if(tmp >= HighLevel) {
			// Rising transition fount
			currentstate = 1;
			risingcnt++;
			risingtime = index;
			if(risingcnt == 1) {
				// Mark down the start of valid sample
				firstrising = index;
				}
			}
		
		}
	index++;
 	}

 
 if((risingcnt >= 2) || (fallingcnt >= 2)) {
 	// At least one cycle found
 	BitSet(Flags, (1 << Flag_FreqValid));
 	
	// Calculate frequency
	if(currentstate == 1) {
		// The last transition is rising. Use first rising as starting point
		sum = risingtime - firstrising;
		risingcnt -= 1;			// Number of cycles
	 	}
	else {
		// The last transition is falling. Use first falling as starting point
		sum = fallingtime - firstfalling;
		risingcnt = fallingcnt - 1;	// Number of cycles
		if(firstrising < firstfalling) {
			// The first crossover is rising. Take out the first segment of high level from 'highleveltime'
			highleveltime -= (firstfalling - firstrising);
			}
	 	}

 	Freq = ((float)risingcnt/(float)sum) * SampleRate[GetTimebase() - TBMin];
	Cycle = 1.0/Freq;
	PW = ((float)highleveltime/(float)risingcnt) / SampleRate[GetTimebase() - TBMin];
	Duty = ((float)highleveltime/(float)sum) * 100.0;	// In percentage
	// Select frequrency unit
	FreqUnit = 0;
	while(Freq >= 1000.0) {
		Freq /= 1000.0;
		FreqUnit++;
		}
	// Select cycle unit
	CycleUnit = 4;
	while(Cycle <= 0.01) {
		Cycle *= 1000.0;
		PW *= 1000.0;
		CycleUnit++;
		}
 	}
 else {
 	BitClr(Flags, (1 << Flag_FreqValid));
 	}
}

void	OnScreenDisplay(void)
{
 U16 tmp1;
		
 // Display measurement results
 tmp1 = clMeasurement;
 if(!BitTest(Flags, (1 << Flag_VoltValid))) {
	tmp1 = clRed;
				}
			
 PutsGenic(VoltageReadingx0, VoltageReadingy0, (U8 *)"Vmax:", clMeasurement, clBlack, &ASC8X16);
 PutsGenic(VoltageReadingx0 + 42, VoltageReadingy0, VoltToStr(Vmax, ReadingStr), tmp1, clBlack, &ASC8X16);
	
 PutsGenic(VoltageReadingx0, VoltageReadingy0 + 1 * 16, (U8 *)"Vmin:", clMeasurement, clBlack, &ASC8X16);
 PutsGenic(VoltageReadingx0 + 42, VoltageReadingy0 + 1 * 16, VoltToStr(Vmin, ReadingStr), tmp1, clBlack, &ASC8X16);
		
 PutsGenic(VoltageReadingx0, VoltageReadingy0 + 2 * 16, (U8 *)"Vavr:", clMeasurement, clBlack, &ASC8X16);
 PutsGenic(VoltageReadingx0 + 42, VoltageReadingy0 + 2 * 16, VoltToStr(Vavr, ReadingStr), tmp1, clBlack, &ASC8X16);

 PutsGenic(VoltageReadingx0, VoltageReadingy0 + 3 * 16, (U8 *)"Vpp:", clMeasurement, clBlack, &ASC8X16);
 PutsGenic(VoltageReadingx0 + 42, VoltageReadingy0 + 3 * 16, VoltToStr(Vpp, ReadingStr), tmp1, clBlack, &ASC8X16);

 PutsGenic(VoltageReadingx0, VoltageReadingy0 + 4 * 16, (U8 *)"Vrms:", clMeasurement, clBlack, &ASC8X16);
 PutsGenic(VoltageReadingx0 + 42, VoltageReadingy0 + 4 * 16, VoltToStr(Vrms, ReadingStr), tmp1, clBlack, &ASC8X16);

 PutsGenic(FreqReadingx0, FreqReadingy0, (U8 *)"Freq:", clMeasurement, clBlack, &ASC8X16);
 PutsGenic(FreqReadingx0, FreqReadingy0 + 1 * 16, (U8 *)"Cycl:", clMeasurement, clBlack, &ASC8X16);
 PutsGenic(FreqReadingx0, FreqReadingy0 + 2 * 16, (U8 *)"PW:", clMeasurement, clBlack, &ASC8X16);
 PutsGenic(FreqReadingx0, FreqReadingy0 + 3 * 16, (U8 *)"Duty:", clMeasurement, clBlack, &ASC8X16);
 if(BitTest(Flags, (1 << Flag_FreqValid))) {
	FloatToStr(Freq, 3, 3, ReadingStr);
	strcat(ReadingStr, UnitsStr[FreqUnit]);
	if((tmp1 = strlen(ReadingStr)) < FreqStrLen) {
		// Clear previous display
		FillRect(FreqReadingx0 + 42, FreqReadingy0, FreqStrLen * 8, 16, clBlack);
		}
	FreqStrLen = tmp1;
	PutsGenic(FreqReadingx0 + 42, FreqReadingy0, ReadingStr, clMeasurement, clBlack, &ASC8X16);
			
	FloatToStr(Cycle, 3, 3, ReadingStr);
	strcat(ReadingStr, UnitsStr[CycleUnit]);
	if((tmp1 = strlen(ReadingStr)) < CycleStrLen) {
		// Clear previous display
		FillRect(FreqReadingx0 + 42, FreqReadingy0 + 1 * 16, CycleStrLen * 8, 16, clBlack);
		}
	CycleStrLen = tmp1;
	PutsGenic(FreqReadingx0 + 42, FreqReadingy0 + 1 * 16, ReadingStr, clMeasurement, clBlack, &ASC8X16);

	FloatToStr(PW, 3, 3, ReadingStr);
	strcat(ReadingStr, UnitsStr[CycleUnit]);
	if((tmp1 = strlen(ReadingStr)) < PwStrLen) {
		// Clear previous display
		FillRect(FreqReadingx0 + 42, FreqReadingy0 + 2 * 16, PwStrLen * 8, 16, clBlack);
		}
	PwStrLen = tmp1;
	PutsGenic(FreqReadingx0 + 42, FreqReadingy0 + 2 * 16, ReadingStr, clMeasurement, clBlack, &ASC8X16);

	FloatToStr(Duty, 3, 1, ReadingStr);
	strcat(ReadingStr, " %");
	PutsGenic(FreqReadingx0 + 42, FreqReadingy0 + 3 * 16, ReadingStr, clMeasurement, clBlack, &ASC8X16);
	BitClr(Flags, (1 << Flag_FreqAreaCleared));
	}
 else {
	if(!BitTest(Flags, (1 << Flag_FreqAreaCleared))) {
		FillRect(FreqReadingx0 + 42, FreqReadingy0, 120, 70, clBlack);
		BitSet(Flags, (1 << Flag_FreqAreaCleared));
		}
	PutsGenic(FreqReadingx0 + 42, FreqReadingy0, (U8 *)"----", clMeasurement, clBlack, &ASC8X16);
	PutsGenic(FreqReadingx0 + 42, FreqReadingy0 + 1 * 16, (U8 *)"----", clMeasurement, clBlack, &ASC8X16);
	PutsGenic(FreqReadingx0 + 42, FreqReadingy0 + 2 * 16, (U8 *)"----", clMeasurement, clBlack, &ASC8X16);
	PutsGenic(FreqReadingx0 + 42, FreqReadingy0 + 3 * 16, (U8 *)"----", clMeasurement, clBlack, &ASC8X16);
	}

				
 if(BitTest(AddOns, (1 << AO_TestSigAmpDisp))) {
	// Display test signal amplitude
	if(BitTest(AddOns, (1 << AO_TestSigAmp))) {
	// Amp is 0.1V
	PutsGenic(TestSigAmpx0, TestSigAmpy0, "T.S. Amp: 0.1V", clWhite, clBlue, &ASC8X16);				
	}
 	else {
		// Amp is normal (3.3V)
		PutsGenic(TestSigAmpx0, TestSigAmpy0, "T.S. Amp: 3.3V", clWhite, clBlue, &ASC8X16);				
		}
	}
 
}

void	TestSigAmpDisplay(void)
{
 // Display test signal amplitude
 if(BitTest(AddOns, (1 << AO_TestSigAmp))) {
 	// Amp is 0.1V
	PutsGenic(TestSigAmpx0, TestSigAmpy0, "T.S. Amp: 0.1V", clWhite, clBlue, &ASC8X16);				
	}
 else {
	// Amp is normal (3.3V)
	PutsGenic(TestSigAmpx0, TestSigAmpy0, "T.S. Amp: 3.3V", clWhite, clBlue, &ASC8X16);				
	}

}
// ---------------------------------------------------------------
// Convert voltage to string
//
U8	*VoltToStr(S16 volt, U8 *strbuf)
{
 float ftmp;
 U8	tmp0;
 U16	tmp1;
 S8	tmp2;

 // Get current Vsen
 tmp2 = GetVSen();
 
 // Calculate voltage. The unit of ftmp is mV
 ftmp = ((float)(volt >> 2) * (3300.0/4096.0)) * UnitRate[tmp2 - VSenMin] * 1.007;
 if(ftmp >= 0) {
	tmp0 = ' ';
	tmp1 = (U16)ftmp;
	}
 else {
	tmp0 = '-';
	tmp1 = (U16)(-ftmp);
	}
 BinToDecStr16(tmp1, strbuf + 1);

 if(tmp2 <= VS_01V) {
	// Unit 'V"
	*(strbuf + 5) = *(strbuf + 4);
	*(strbuf + 4) = *(strbuf + 3);
	*(strbuf + 3) = '.';
	*(strbuf + 6) = 'V';
	*(strbuf + 7) = 0;

	// Remove leading 0's
	if(*(strbuf + 1) == '0') {
		*(strbuf + 1) = tmp0;
		*(strbuf + 0) = ' ';
//		tmp2 = 1;
		}
	else {
		*(strbuf + 0) = tmp0;		// Sign		
//		tmp2 = 0;
		}
 	}
 else {
 	*(strbuf + 0) = ' ';
 	*(strbuf + 1) = *(strbuf + 2);
 	*(strbuf + 2) = *(strbuf + 3);
 	*(strbuf + 3) = *(strbuf + 4);
 	*(strbuf + 4) = *(strbuf + 5);
 	*(strbuf + 5) = 'm';
 	*(strbuf + 6) = 'V';
 	*(strbuf + 7) = 0;
	// Remove leading 0's
	tmp2 = 1;
	while(tmp2 < 4) {
		if(*(strbuf + tmp2) == '0') {
			*(strbuf + tmp2) = ' ';
			}
		else {
			break;
			}
		tmp2++;
		}
	tmp2--;
	*(strbuf + tmp2) = tmp0;		// Sign
 	}

 return strbuf;
}

U8 	*BinToDecStr16(U16 binary, U8 *Str)
{
 U8	 tmp0;

 tmp0 = binary/10000;
 *(Str + 0) = 0x30 | tmp0;
 binary = binary - tmp0 * 10000;
 
 tmp0 = binary/1000;
 *(Str + 1) = 0x30 | tmp0;
 binary = binary - tmp0 * 1000;

 tmp0 = binary/100;
 *(Str + 2) = 0x30 | tmp0;
 binary = binary - tmp0 * 100;

 tmp0 = binary/10;
 *(Str + 3) = 0x30 | tmp0;
 binary = binary - tmp0 * 10;
 
 *(Str + 4) = 0x30 | binary;

 *(Str + 5) = 0;

 return Str;

}

// ------------------------------------------------------------------------
// Convert float number 'f' to string with 'width' digits of whole and 'precision' digits of fraction
//	digits. Return pointer to the string.
// Condition: f < 10^10, width < 10, presicion < 10
U8 	*FloatToStr(float f, U8 width, U8 precision, U8 *str)
{
 U32 tmp0;
 U8	tmp1, tmp3, tmp4;
 U8	tmpstr[24], *ptmp;

 tmp1 = 0;
 if(f < 0.0) {
 	tmp1 = 1;	// Negative
 	f = 0.0 - f;
 	}

 tmp0 = f;			// Whole portion
 BinToDec32(tmp0, &tmpstr[1]);
 
 tmp3 = 0;
// Looking for the first non-zero digit
 while(tmp3 < 10) {
	if(tmpstr[tmp3 + 1] != '0') {
		break;
		}
	tmp3++;
 	}
 DeZero(&tmpstr[tmp3 + 1], width, ' ');
 if(tmp3 == 10) {
 	tmpstr[10] = '0';		// Whole part is zero. Keep one '0' before decimal point
 	tmp3 = 9;
 	}
 
 // Add '-' for negative number
 if(tmp1) {
	tmpstr[tmp3] = '-';
 	}
 else {
 	tmpstr[tmp3] = ' ';
 	}

 if(precision == 0) {
 	// No fraction required
	tmpstr[11] = 0;
 	}
 else {
 	tmpstr[11] = '.';
	f = f - (float)tmp0;	// Get fraction
	tmp4 = precision;
	while(tmp4) {
		f *= 10.0;
		tmp4--;
	 	}
	tmp0 = f;			// Fraction
	BinToDec32(tmp0, &tmpstr[12]);
	// Advance the digits of interest
	tmp4 = 0;
	while(tmp4 < precision) {
		tmpstr[12 + tmp4] = tmpstr[12 + (10 - precision) + tmp4];
		tmp4++;
	 	}
	tmpstr[12 + tmp4] = 0;
 	}
 
 ptmp = str;
 tmp4 = 0;
 while((*ptmp++ = tmpstr[tmp3 + tmp4])) {
 	tmp4++;
 	}
 return str;
}

U8 	*BinToDec32(U32 binary, U8 *Str)
{
 U8	 tmp0;
 tmp0 = 0;
 while(tmp0 <= 9) {
	binary = FindDec(binary, tmp0, (U8 *)Str);
	tmp0++;
 	}

 *(Str + 10) = 0;

 return Str;

}

U32 FindDec(U32 bin, U8 ind, U8 *str)
{
 U8 tmp;
 U32	tmp1;

 tmp1 = 1;
 tmp = 9 - ind;
 while(tmp) {
	tmp1 *= 10;
	tmp--;
 	}

 tmp = bin/tmp1;
 *(str + ind) = 0x30 | tmp;
 return (bin - tmp * tmp1);
}

// Remove leading zeros for the first 'num' digits. Replace them with 'ch'
void	DeZero(U8 *str, U8 num, U8 ch)
{
 U8 tmp;

 tmp = 0;
 while(tmp < num) {
	if(*str == '0') {
		*str = ch;
		}
	else {
		break;
		}
	str++;
 	}

 // Put '0' back if '.' is the first non-zero digit
 if(*str == '.') {
 	str--;
	*str = '0';
 	}
}

