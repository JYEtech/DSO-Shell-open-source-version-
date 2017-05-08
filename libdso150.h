//////////////////////////////////////////////////////////////////////////////
//
// 	Filename:	libdso138.h
//	Version:		
//	Data:		
//
//	Author:		Liu, Zemin
//	Company:	JYE Tech Ltd.
//	Web:		www.jyetech.com
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

#ifndef	libdso150_h

#define	libdso150_h

//#include	"Common.h"
#include	"Screen.h"

// =======================================================
#define	SampleBufSizeMax	0x400

#define	VPosMax	500
#define	VPosMin		-500

#define	VSenMax	VS_5mV
#define	VSenMin		VS_20V

#define	CplMax		CP_GND
#define	CplMin		CP_DC

#define	TBMax		TB_10us
#define	TBMin		TB_500s

#define	TrigModeMax	TM_Single
#define	TrigModeMin		TM_Auto

#define	TrigEdgeMax	TE_Rising
#define	TrigEdgeMin		TE_Falling

#define	TrigLvlMax		1200
#define	TrigLvlMin		-1200

#define	TrigPosMax		SampleBufSizeMax
#define	TrigPosMin		0

#define	TrigSenMax		100
#define	TrigSenMin		0

#define	RecLenMax		SampleBufSizeMax
#define	RecLenMin		512

#define	SampleMidValue		0x800		// For 1.65V bias
#define	WWindowMidValue	(SampleMidValue >> 2)		

// =======================================================
//	Scope control type definitions
// =======================================================
//

// -- Time base type
enum	TimeBase {
//	TB_min,
	TB_Custom 	= 0,	
	TB_500s,		
	TB_200s,		
	TB_100s,		
	TB_50s,		
	TB_20s,		
	TB_10s,		
	TB_5s,		
	TB_2s,		
	TB_1s,		
	TB_05s,		
	TB_02s,		
	TB_01s,		
	TB_50ms,	
	TB_20ms,	
	TB_10ms,	
	TB_5ms,		
	TB_2ms,		
	TB_1ms,		
	TB_05ms,	
	TB_02ms,	
	TB_01ms,	
	TB_50us,	
	TB_20us,	
	TB_10us,	
	TB_5us,		
	TB_2us,		
	TB_1us,		
	TB_05us,	
	TB_02us,
	TB_01us,
	TB_50ns,
	TB_20ns,
	TB_10ns,
//	TB_max
	};

// -- Trig Mode type
enum TrigMode	{
	TM_Auto,
	TM_Normal,
	TM_Single,
	TM_Max
	};


// -- Trig Edge type
enum TrigEdge {
	TE_Falling,
	TE_Rising,
	};

// -- V sensitivity type
typedef	enum 	{
	VS_UnCal		= 0,
	VS_50V,
	VS_20V,
	VS_10V,
	VS_5V,
	VS_2V,
	VS_1V,
	VS_05V,
	VS_02V,
	VS_01V,
	VS_50mV,
	VS_20mV,
	VS_10mV,
	VS_5mV,
	VS_Last
	}TY_VSen ;

// -- Coupling type
enum 	Couple{
	CP_DC,
	CP_AC,
	CP_GND
	};

enum FocusType {
	FC_VSen		= 0,
	FC_Timebase,
	FC_TrigMode,
	FC_TrigEdge,
	FC_TrigLvl,
	FC_HPos,
	FC_VPos,
	FC_Max

};


enum DsoStatus {
	DSO_CaptureDone 		= 0x0001,
	DSO_Trigged 			= 0x0002,
	DSO_Hold 				= 0x0004,
	DSO_Rolling 				= 0x0008,
	DSO_NormalDisplay		= 0x0080,
};

enum DispUpdateFlags {
	Disp_Panel 		= 0x0001,
	Disp_Param 		= 0x0002,
	Disp_Trace 		= 0x0004,
	Disp_None		= 0x0008,
	Disp_TrState	= 0x0010,	// Display trigger state
};

// =====================================
// Exported variables
//
extern	U16	*SampleBuf;
extern	U16	*CurrentSample;
extern	const	U8	LibVersion[];

extern	FONT JYELogo;
extern	FONT Jinyuedianzi;
extern	FONT Jyetech;
extern	FONT Web;
extern	FONT ByJyetech;

// -- Function Prototypes --
void	DSO_Init(void);
void	StartCapture(void);
void	StopCapture(void);
U16	GetDsoStatus(void);
void	SetHold(void);
void	ClrHold(void);
void	UpdateTimebase(void);
void Rolling(void);

S8	SetVSen(S8 vsen);
S8	SetCpl(S8 cpl);
S16	SetVPos(S16 vpos);
S8	SetTimeBase(S8 timebase);
S16	SetHPos(S16 hpos);
S8	SetTrigMode(S8 trigmode);
S8	SetTrigEdge(S8 trigslope);
S16 SetTrigLvl(S16 triglvl);
S16 SetTrigPos(S16 trigpos);
S16 	SetTrigSen(S16 trigsen);
U16	SetRecLen(U16 reclen);
S16	SetVPosOfs(S16 ofs);

S8 	GetVSen(void);
S8 	GetCpl(void);
S16 	GetVPos(void);
S8 	GetTimebase(void);
S16 	GetHPos(void);
S8 	GetTrigMode(void);
S8 	GetTrigEdge(void);
S16 	GetTrigLvl(void);
S16 GetTrigPos(void);
S16 	GetTrigSen(void);
U16 	GetRecLen(void);
S16	GetVPosOfs(void);

void	DsoDisplay(void);
void	UpdateDisp(U16 disp);
void	CancelDisp(U16 disp);
U8	SetFocus(U8 focus);
U8	GetFocus(void);

S16	GetAverage(void);

#endif

