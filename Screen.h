//////////////////////////////////////////////////////////////////////////////
//
// 	Filename:	Board.h
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
//

#ifndef Screen_h

#define	Screen_h

#include	"Common.h"

// ------------ Display parameters -----------------------
// Overall display parameters
#define		ScreenX0 		0
#define		ScreenY0		0
#define		ScreenXsize		320
#define		ScreenYsize		240

#define		Rbits			0		// Red bits position
#define		Gbits			5		// Green bits position
#define		Bbits			11		// Blue bits position
#define		RGB(R, G, B)	(((R >> 3) << Rbits) | ((G >> 2) << Gbits) | ((B >> 3) << Bbits))

enum  COLOR{
	clBlack 			=	RGB(0, 0, 0),
	clWhite			= 	RGB(255, 255, 255),
	clRed			= 	RGB(255, 0, 0),
	clGreen			= 	RGB(0, 255, 0),
	clBlue			= 	RGB(0, 0, 255),
	clYellow			=	RGB(255, 255, 0),
	clGainsboro		=	RGB(220, 220, 220),
	clNavy			=	RGB(0, 0, 128),
	clAqua			=	RGB(0, 255, 255),
	clHotpink		=	RGB(255, 105, 180),
	clOrange		=	RGB(255, 165, 0),
	clDeepskyblue	=	RGB(0, 191, 255),
	clDimgray		=	RGB(105, 105, 105),
	cllightsalmon	=	RGB(255, 160, 122),
	cllightcoral		=	RGB(240, 128, 128),
	clpaleturquoise	=	RGB(175, 238, 238),
	clturquoise		=	RGB(64, 224, 208),
	clViolet			=	RGB(238, 130, 238),
	clGray1			=	RGB(90, 90, 90),
	clGray2			=	RGB(220, 220, 220),
	clGray3			=	RGB(240, 240, 240),
	clDarkgray		=	RGB(169, 169, 169),
	clSkyblue		= 	RGB(135, 206, 235),
	clChocolate		= 	RGB(210, 105, 30),
	clMediumseagreen	= 	RGB(60, 179, 113),
	clPeachpuff		=	RGB(255, 218, 185),
	clSeagreen		=	RGB(46, 139, 87),

	clBG1			= 	clGainsboro,
	
	clActiveItem1	=	clAqua,
	clActiveItem2	=	clturquoise,

	clBtnBG1		=	clOrange,
	clBtnBG2		= 	clBlue,
	clBtnBG3		=	clGainsboro,
	clBtnBG4		= 	clSkyblue,
	clBtnBG5		= 	clRed,
	
	clBtnFG1		=	clBlack,
	clBtnFG2		= 	clWhite,

	clCh1			= 	clYellow,
	clTB				=	clGreen,
	clTrigger		=	clHotpink,

	clCursorT		= 	clMediumseagreen,
	clCursorV		= 	clOrange,
	clCursorActive	=	clAqua,
	clMeasurement	= 	clGray3,
} ;

typedef struct {
	U8 	*Array;
	U16	Xsize;
	U8	Ysize;
	U16	CharPitch;
	U8	LinePitch;
	U8	IndexOfs;
	} FONT;


//	Display Panel
// =======================================================
// Display parameters
#define	WWindowx0			10
#define	WWindowy0			17
#define	WWindowSizex		300
#define	WWindowSizey		200

#define	WDsize				300		// Wave display size
#define	HBarSize			140		// HPos indicator length

// ===========================================================
//	Declarations of external variables
// ===========================================================
//
extern	FONT DSOm;
extern	FONT Shell;
extern	FONT DSO_Shell;
extern	FONT ASC8X16;

// ===========================================================
//	Function Prototype Declarations
// ===========================================================
//
void	ClrScreen(void);
void	SetWindow(U16 x, U16 xsize, U16 y, U16 ysize);
void FillRect(S16 x, S16 y, S16 xsize, S16 ysize, U16 color);
void	PutcGenic(U16 x, U16 y, U8 ch, U16 fgcolor, U16 bgcolor, FONT *font);
void	PutsGenic(U16 x, U16 y, U8 *str, U16 fgcolor, U16 bgcolor, FONT *font);

#endif

