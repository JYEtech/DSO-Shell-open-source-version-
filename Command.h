//////////////////////////////////////////////////////////////////////////////
//
// 	Filename:	Command.h
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
//
//
#ifndef	Command_h
#define	Command_h

#include	"Common.h"

// ============== Key Analysis Definitions =====================

typedef	struct {
	// Keypad processing variables
	U16	Flags;
	U16	KScanBuf;
	U16 	KScanCode;
	U16	KeyCode;
	U16	KeyParam;
	U16	KeyCodeBuf;
	U16	KCount;			// Key debounce counter
	U16	KDebounceVal;	// Debounce timing length (number of scans)
	U16	KHCount;		// Key hold counter
	U16	KTimeChk;		// Key time check 
	U16	EncState;

	} KEYPAD;

enum	KeypadFlags {
	KF_DoKeyScan		= 0,
	KF_PressDetected	= 1,
	KF_HoldDetected		= 2,
	KF_KeyHit			= 3,

	};

// -- Key-Event mapping 
typedef	struct	{
	U8	Keycode;
	U8	Event;
	}KeyEvent ;

// -- Key code conversion 
typedef	struct	{
	U16 	ScanCode;
	U8	Keycode;
	}KeyMap ;

// --------------------------------------
// Keypad 
typedef	struct	{
	U16	ScanCode;
	U8	KeyCode;
	U8	KeyParam;
	}KeyScanCode;

// -- Key Code Definitions 
enum	KeyCodes {
	KC_void = 0,				
	KC_SW1,				
	KC_SW2,				
	KC_SW3,				
	KC_SW4,				
	KC_SW1H,				
	KC_SW2H,				
	KC_SW3H,				
	KC_SW4H,	
	KC_SW1_SW4,
	KC_SW2_SW4,
	KC_SW3_SW4,
	KC_SW2_SW3,
	KC_SW2_SW3H,
	KC_ENC_CW,
	KC_ENC_CCW,
	KC_ENC_PB,
	KC_ENC_PBH,
	KC_PB_SW1,
	KC_PB_SW2,
	KC_PB_SW3,
	KC_PB_SW4,
	};

// Pushbutton processing parameters
#define	NoKey			0xFFFF	

#define	KD_val			6		// 
#define	KD_val1			15		// 
#define	KH_val			90		// 
#define	KR_Time		3	

// Add-ons
enum	AddOn {
	AO_MeasurementOn 		= 0,
	AO_TestSigAmp			= 1,		// 0 - normal (3.3V), 1 - 0.1V
	AO_TestSigAmpDisp		= 2,

};

// Setting status
#define	SettingStatus_Initialized		0xF3A5

#define	VoltageReadingx0	205
#define	VoltageReadingy0	20
#define	FreqReadingx0		15
#define	FreqReadingy0		20
#define	TestSigAmpx0		15
#define	TestSigAmpy0		(WWindowy0 + WWindowSizey - 20)

#define	Flag_MeasurementOn		0
#define	Flag_FreqValid				1
#define	Flag_VoltValid				2
#define	Flag_FreqAreaCleared		3

#define Waveform_START_ADDRESS    ((uint32_t)0x0800E000) 
#define Parameter_START_ADDRESS    ((uint32_t)0x0800E800) 

// ===========================================================
//	Export variables
// ===========================================================
//
extern	U16	AddOns;	
extern	KEYPAD	Keypad;
extern	S16	Vmax;
extern	S16	Vmin;
extern	S16	Vavr;
extern	S16	Vpp;
extern	S16	Vrms;
extern	U8	ReadingStr[];
extern	U8	MeasurementOn;
extern	float	Freq;
extern	float	Cycle;
extern	float	PW;
extern	float	Duty;
extern	U16	Flags;
extern	U16	FreqUnit;
extern	U16	CycleUnit;
extern	U16	FreqStrLen;
extern	U16	CycleStrLen;
extern	U16	PwStrLen;
extern	U8	*UnitsStr[8];

// ===========================================================
//	Function Declarations
// ===========================================================
//
void	AppInit();
void	LoadDefault(void);
void KeyScan(void);
void	KeyConvert(KeyScanCode *KSCTab, U16 KSCode);
U8	KeyEventMap(U8 keycode, KeyEvent *kvmap);
void	KeyProc(void);
void	DoKeyOk(void);
void	DoKeyInc(void);
void	DoKeyDec(void);
void	DoKeySel(void);
void	DoKeyOkH(void);
void	DoKeyIncH(void);
void	DoKeyDecH(void);
void	DoKeySelH(void);
void	SelVertical(void);
void SelHorzontal(void);
void	SelTrigger(void);
void	VPosAlign(void);
void	CenterHPos(void);
void	CenterTrigLevel(void);
void	LedBlink(void);
void	LedBlink_Fast(void);
void	TestMode(void);
void	SaveWaveform(void);
void	LoadWaveform(void);
void	Measurements(void);
void	OnScreenDisplay(void);
void	TestSigAmpDisplay(void);
U8	*VoltToStr(S16 volt, U8 *strbuf);
U8 	*BinToDecStr16(U16 binary, U8 *Str);
U8 	*BinToDec32(U32 binary, U8 *Str);
U32 FindDec(U32 bin, U8 ind, U8 *str);
void	DeZero(U8 *str, U8 num, U8 ch);
U8 	*FloatToStr(float f, U8 width, U8 precision, U8 *str);


#endif

