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
//

#ifndef	Board_h

#define	Board_h

#include	"Common.h"

#include "stm32f10x.h"
//#include "stm32f10x_conf.h"

// TFT control ports

#define	TFT_nRESET_Port			GPIOB	
#define	TFT_nRESET_Bit				9		
#define	TFT_RS_Port					GPIOC
#define	TFT_RS_Bit					14	
#define	TFT_nCS_Port				GPIOC
#define	TFT_nCS_Bit					13	
#define	TFT_nWR_Port				GPIOC
#define	TFT_nWR_Bit				15	
#define	TFT_nRD_Port				GPIOA
#define	TFT_nRD_Bit					6	

#define	TFT_Port					(GPIOB->ODR)
#define	TFT_Port_In					(GPIOB->IDR)

#define	LED_Base					GPIOA
#define	LED_Port					(GPIOA->ODR)
#define	LED_Bit						15

#define	Beep_Base					(GPIOA)
#define	Beep_Port					(GPIOA->ODR)
#define	Beep_Bit					15

#define	I2C_SCL_Port				GPIOB
#define	I2C_SCL_Bit					10
#define	I2C_SDA_Port				GPIOB
#define	I2C_SDA_Bit					11

// I2C Macros
#define	SetSCL_H					(I2C_SCL_Port->BSRR = 1 << I2C_SCL_Bit)
#define	SetSCL_L					(I2C_SCL_Port->BRR = 1 << I2C_SCL_Bit)
#define	SetSDA_H					(I2C_SDA_Port->BSRR = 1 << I2C_SDA_Bit)
#define	SetSDA_L					(I2C_SDA_Port->BRR = 1 << I2C_SDA_Bit)
#define	GetSDA						(BitTest(I2C_SDA_Port->IDR, (1 << I2C_SDA_Bit)))

// Pushbuttons and rotary encoder
#define	PB_Port						(GPIOB->IDR)
#define	PB_Bits						0x00F8
#define	ENC_Port					(GPIOB->IDR)
#define	ENC_Bits					0x0003

// VSen control
#define	VSen_Port					(GPIOA->ODR)
#define	VSen_Bits					0x001E

// ILI9325 command registers
#define	TFT_DriverOutputControl		0x0001
#define	TFT_DrivingWaveControl		0x0002
#define	TFT_EntryMode				0x0003
#define	TFT_DisplayControl1			0x0007
#define	TFT_DisplayControl2			0x0008
#define	TFT_DisplayControl3			0x0009
#define	TFT_DisplayControl4			0x000A
#define	TFT_FrameMarkerPosition	0x000D
#define	TFT_PowerControl1			0x0010
#define	TFT_PowerControl2			0x0011
#define	TFT_PowerControl3			0x0012
#define	TFT_PowerControl4			0x0013
#define	TFT_DramHAddress			0x0020
#define	TFT_DramVAddress			0x0021
#define	TFT_DramDataWrite			0x0022
#define	TFT_DramDataRead			0x0022
#define	TFT_VCOMH_Control			0x0029
#define	TFT_FrameRateColor			0x002B
#define	TFT_HAddressStart			0x0050
#define	TFT_HAddressEnd			0x0051
#define	TFT_VAddressStart			0x0052
#define	TFT_VAddressEnd			0x0053
#define	TFT_GateScanControl1		0x0060
#define	TFT_GateScanControl2		0x0061
#define	TFT_Panel_IF_Control1		0x0090
#define	TFT_Panel_IF_Control2		0x0092
#define	TFT_FAh_FEh_Enable		0x00FF

// ======== STM32 Register Constants =====================

// -------- Register address -----------------
// RCC registers
#define RCC_AHBENR (*((unsigned int *)(0x40021014)))
#define RCC_APB2ENR (*((unsigned int *)(0x40021018)))
#define RCC_APB1ENR (*((unsigned int *)(0x4002101C)))

// GPIO registers
#define GPIOA_CRL   (*((unsigned int *)(0x40010800)))
#define GPIOA_BSRR  (*((unsigned int *)(0x40010810)))
#define GPIOA_BRR   (*((unsigned int *)(0x40010814)))

#define GPIOB_CRL   (*((unsigned int *)(0x40010C00)))
#define GPIOB_CRH   (*((unsigned int *)(0x40010C04)))
#define GPIOB_IDR   (*((unsigned int *)(0x40010C08)))
#define GPIOB_ODR   (*((unsigned int *)(0x40010C0C)))
#define GPIOB_BSRR  (*((unsigned int *)(0x40010C10)))
#define GPIOB_BRR   (*((unsigned int *)(0x40010C14)))
#define GPIOB_LCKR   (*((unsigned int *)(0x40010C18)))

#define GPIOD_CRL   (*((unsigned int *)(0x40011400)))
#define GPIOD_CRH   (*((unsigned int *)(0x40011404)))
#define GPIOD_IDR   (*((unsigned int *)(0x40011408)))
#define GPIOD_ODR   (*((unsigned int *)(0x4001140C)))
#define GPIOD_BSRR  (*((unsigned int *)(0x40011410)))
#define GPIOD_BRR   (*((unsigned int *)(0x40011414)))
#define GPIOD_LCKR   (*((unsigned int *)(0x40011418)))

#define GPIOE_CRL   (*((unsigned int *)(0x40011800)))
#define GPIOE_CRH   (*((unsigned int *)(0x40011804)))
#define GPIOE_IDR   (*((unsigned int *)(0x40011808)))
#define GPIOE_ODR   (*((unsigned int *)(0x4001180C)))
#define GPIOE_BSRR  (*((unsigned int *)(0x40011810)))
#define GPIOE_BRR   (*((unsigned int *)(0x40011814)))
#define GPIOE_LCKR   (*((unsigned int *)(0x40011818)))

// FSMC registers
#define FSMC_BCR1   (*((U32 *)(0xA0000000)))
#define FSMC_BTR1   (*((U32 *)(0xA0000004)))
#define FSMC_BWTR1   (*((U32  *)(0xA0000104)))

#define FSMC_BCR2   (*((U32  *)(0xA0000008)))
#define FSMC_BTR2   (*((U32  *)(0xA000000C)))
#define FSMC_BWTR2   (*((U32  *)(0xA000010C)))

// ---------------- Bit fields ------------------------
// Clock control
//-- AHBENR
#define	SDIOEN			10
#define	FSMCEN			8
#define	CRCEN			6
#define	FLITFEN			4
#define	SRAMEN			2
#define	DMA2EN			1
#define	DMA1EN			0

//-- APB1ENR
#define	DACEN			29
#define	PWREN			28
#define	BKPEN			27
#define	CANEN			25
#define	USBEN			23
#define	I2C2EN			22
#define	I2C1EN			21
#define	UART5EN		20
#define	UART4EN		19
#define	USART3EN		18
#define	USART2EN		17

#define	SPI3EN			15
#define	SPI2EN			14
#define	WWDGEN		11
#define	TIM7EN			5
#define	TIM6EN			4
#define	TIM5EN			3
#define	TIM4EN			2
#define	TIM3EN			1
#define	TIM2EN			0

//-- APB2ENR
#define	ADC3EN			15
#define	USART1EN		14
#define	TIM8EN			13
#define	SPI1EN			12
#define	TIM1EN			11
#define	ADC2EN			10
#define	ADC1EN			9
#define	IOPGEN			8
#define	IOPFEN			7
#define	IOPEEN			6
#define	IOPDEN			5
#define	IOPCEN			4
#define	IOPBEN			3
#define	IOPAEN			2
#define	AFIOEN			0


// ---------------- Bit fields ------------------------
// Clock control
//
/********************  Bit definition for RCC_CR register  ********************/
#define  HSION             	0		/*!< Internal High Speed clock enable */
#define  HSIRDY            	1		/*!< Internal High Speed clock ready flag */
#define  HSITRIM          	3		/*!< Internal High Speed clock trimming */
#define  HSICAL            	8		/*!< Internal High Speed clock Calibration */
#define  HSEON              	16		/*!< External High Speed clock enable */
#define  HSERDY             	17		/*!< External High Speed clock ready flag */
#define  HSEBYP            	18		/*!< External High Speed clock Bypass */
#define  CSSON           		19		/*!< Clock Security System enable */
#define  PLLON             	24		/*!< PLL enable */
#define  PLLRDY          		25		/*!< PLL clock ready flag */

/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define  SW                 	0		/*!< SW[1:0] bits (System clock Switch) */

/*!< SWS configuration */
#define  SWS                  	2		/*!< SWS[1:0] bits (System Clock Switch Status) */

/*!< HPRE configuration */
#define  HPRE                  	4		/*!< HPRE[3:0] bits (AHB prescaler) */

/*!< PPRE1 configuration */
#define  PPRE1                 	8		/*!< PRE1[2:0] bits (APB1 prescaler) */

/*!< PPRE2 configuration */
#define  PPRE2                 	11		/*!< PRE2[2:0] bits (APB2 prescaler) */

/*!< ADCPPRE configuration */
#define  ADCPRE                	14		/*!< ADCPRE[1:0] bits (ADC prescaler) */

#define  PLLSRC                	16		/*!< PLL entry clock source */

#define  PLLXTPRE             	17		 /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define  PLLMULL              	18		/*!< PLLMUL[3:0] bits (PLL multiplication factor) */

 #define  USBPRE             	22		/*!< USB Device prescaler */

/*!< MCO configuration */
#define  MCO                   	24		/*!< MCO[2:0] bits (Microcontroller Clock Output) */

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define  LSIRDYF             	0		 /*!< LSI Ready Interrupt flag */
#define  LSERDYF            	1		/*!< LSE Ready Interrupt flag */
#define  HSIRDYF             	2		/*!< HSI Ready Interrupt flag */
#define  HSERDYF            	3		/*!< HSE Ready Interrupt flag */
#define  PLLRDYF             	4		/*!< PLL Ready Interrupt flag */
#define  CSSF                   	7		/*!< Clock Security System Interrupt flag */
#define  LSIRDYIE            	8		 /*!< LSI Ready Interrupt Enable */
#define  LSERDYIE            	9		/*!< LSE Ready Interrupt Enable */
#define  HSIRDYIE           	10		/*!< HSI Ready Interrupt Enable */
#define  HSERDYIE          	11		/*!< HSE Ready Interrupt Enable */
#define  PLLRDYIE            	12		/*!< PLL Ready Interrupt Enable */
#define  LSIRDYC              	16		/*!< LSI Ready Interrupt Clear */
#define  LSERDYC            	17		/*!< LSE Ready Interrupt Clear */
#define  HSIRDYC            	18		/*!< HSI Ready Interrupt Clear */
#define  HSERDYC             	19		/*!< HSE Ready Interrupt Clear */
#define  PLLRDYC            	20		/*!< PLL Ready Interrupt Clear */
#define  CSSC                  	23		/*!< Clock Security System Interrupt Clear */

/*****************  Bit definition for RCC_APB2RSTR register  *****************/
#define  AFIORST              	0		/*!< Alternate Function I/O reset */
#define  IOPARST             	2		/*!< I/O port A reset */
#define  IOPBRST            	3		/*!< I/O port B reset */
#define  IOPCRST             	4		/*!< I/O port C reset */
#define  IOPDRST             	5		 /*!< I/O port D reset */
#define  IOPERST            	6		/*!< I/O port E reset */
#define  IOPFRST             	7		/*!< I/O port F reset */
#define  IOPGRST             	8		/*!< I/O port G reset */
#define  ADC1RST             	9		/*!< ADC 1 interface reset */
#define  ADC2RST            	10		/*!< ADC 2 interface reset */
#define  TIM1RST           	11		/*!< TIM1 Timer reset */
#define  SPI1RST           	12		/*!< SPI 1 reset */
#define  TIM8RST           	13		/*!< TIM8 Timer reset */
#define  USART1RST       	14		/*!< USART1 reset */
#define  ADC3RST             	15		/*!< ADC3 interface reset */

/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define  TIM2RST              	0		/*!< Timer 2 reset */
#define  TIM3RST              	1		/*!< Timer 3 reset */
#define  TIM4RST               2		/*!< Timer 4 reset */
#define  TIM5RST             	3		/*!< Timer 5 reset */
#define  TIM6RST              	4		 /*!< Timer 6 reset */
#define  TIM7RST              	5		/*!< Timer 7 reset */
#define  WWDGRST            	11		/*!< Window Watchdog reset */
#define  SPI2RST             	14		/*!< SPI 2 reset */
#define  SPI3RST             	15		/*!< SPI 3 reset */
#define  USART2RST          	17		/*!< USART 2 reset */
#define  USART3RST          	18		/*!< RUSART 3 reset */
#define  UART4RST            	19		/*!< UART 4 reset */
#define  UART5RST            	20		/*!< UART 5 reset */
#define  I2C1RST             	21		/*!< I2C 1 reset */
#define  I2C2RST           	22		/*!< I2C 2 reset */
#define  USBRST          	23		/*!< USB Device reset */
#define  CAN1RST           	25		/*!< CAN1 reset */
#define  BKPRST             	27		/*!< Backup interface reset */
#define  PWRRST            	28		/*!< Power interface reset */
#define  DACRST              	29		/*!< DAC interface reset */

/******************  Bit definition for RCC_AHBENR register  ******************/
#define  DMA1EN                0		/*!< DMA1 clock enable */
#define  DMA2EN              	1		/*!< DMA2 clock enable */
#define  SRAMEN              	2		 /*!< SRAM interface clock enable */
#define  FLITFEN               	4		/*!< FLITF clock enable */
#define  CRCEN                	6		/*!< CRC clock enable */
#define  FSMCEN              	8		/*!< FSMC clock enable */
#define  SDIOEN               	10		/*!< SDIO clock enable */

/******************  Bit definition for RCC_APB2ENR register  *****************/
#define  AFIOEN                 0		/*!< Alternate Function I/O clock enable */
#define  IOPAEN               	2		/*!< I/O port A clock enable */
#define  IOPBEN              	3		/*!< I/O port B clock enable */
#define  IOPCEN              	4		/*!< I/O port C clock enable */
#define  IOPDEN               	5		/*!< I/O port D clock enable */
#define  IOPEEN                	6		/*!< I/O port E clock enable */
#define  IOPFEN             	7		/*!< I/O port F clock enable */
#define  IOPGEN               	8		/*!< I/O port G clock enable */
#define  ADC1EN                	9		/*!< ADC 1 interface clock enable */
#define  ADC2EN                	10		/*!< ADC 2 interface clock enable */
#define  TIM1EN               	11		/*!< TIM1 Timer clock enable */
#define  SPI1EN                	12		/*!< SPI 1 clock enable */
#define  TIM8EN               	13		/*!< TIM8 Timer clock enable */
#define  USART1EN            	14		/*!< USART1 clock enable */
#define  ADC3EN              	15		/*!< DMA1 clock enable */

/*****************  Bit definition for RCC_APB1ENR register  ******************/
#define  TIM2EN                	0		/*!< Timer 2 clock enabled*/
#define  TIM3EN                	1		/*!< Timer 3 clock enable */
#define  TIM4EN               	2		/*!< Timer 4 clock enable */
#define  TIM5EN            	3		/*!< Timer 5 clock enable */
#define  TIM6EN               	4		/*!< Timer 6 clock enable */
#define  TIM7EN               	5		/*!< Timer 7 clock enable */
#define  WWDGEN              	11		/*!< Window Watchdog clock enable */
#define  SPI2EN              	14		/*!< SPI 2 clock enable */
#define  SPI3EN                	15		/*!< SPI 3 clock enable */
#define  USART2EN           	17		/*!< USART 2 clock enable */
#define  USART3EN           	18		/*!< USART 3 clock enable */
#define  UART4EN             	19		/*!< UART 4 clock enable */
#define  UART5EN            	20		/*!< UART 5 clock enable */
#define  I2C1EN                	21		/*!< I2C 1 clock enable */
#define  I2C2EN               	22		/*!< I2C 2 clock enable */
#define  USBEN               	23		/*!< USB Device clock enable */
#define  CAN1EN                	25		/*!< CAN1 clock enable */
#define  BKPEN                  	27		/*!< Backup interface clock enable */
#define  PWREN                	28		/*!< Power interface clock enable */
#define  DACEN                 	29		/*!< DAC interface clock enable */


/*******************  Bit definition for RCC_BDCR register  *******************/
#define  LSEON              	0		/*!< External Low Speed oscillator enable */
#define  LSERDY               	1		/*!< External Low Speed oscillator Ready */
#define  LSEBYP             	2		/*!< External Low Speed oscillator Bypass */

#define  RTCSEL                 	8		/*!< RTCSEL[1:0] bits (RTC clock source selection) */

#define  RTCEN                   15		/*!< RTC clock enable */
#define  BDRST                	16		/*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CSR register  ********************/  
#define  LSION                  	0		/*!< Internal Low Speed oscillator enable */
#define  LSIRDY              	1		/*!< Internal Low Speed oscillator Ready */
#define  RMVF                    	24		/*!< Remove reset flag */
#define  PINRSTF              	26		/*!< PIN reset flag */
#define  PORRSTF              	27		/*!< POR/PDR reset flag */
#define  SFTRSTF               28		/*!< Software Reset flag */
#define  IWDGRSTF            29		/*!< Independent Watchdog reset flag */
#define  WWDGRSTF        	30		/*!< Window watchdog reset flag */
#define  LPWRRSTF            	31		/*!< Low-Power reset flag */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for TIM_CR1 register  ********************/
#define  CEN                     	0           /*!<Counter enable */
#define  UDIS                   	1            /*!<Update disable */
#define  URS                      	2            /*!<Update request source */
#define  OPM                     	3            /*!<One pulse mode */
#define  DIR                		4            /*!<Direction */

#define  CMS                    	5            /*!<CMS[1:0] bits (Center-aligned mode selection) */

#define  ARPE                    	7            /*!<Auto-reload preload enable */

#define  CKD                     	8            /*!<CKD[1:0] bits (clock division) */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  CCPC                 	0           /*!<Capture/Compare Preloaded Control */
#define  CCUS                   	2            /*!<Capture/Compare Control Update Selection */
#define  CCDS                 	3           /*!<Capture/Compare DMA Selection */

#define  MMS                   	4            /*!<MMS[2:0] bits (Master Mode Selection) */

#define  TI1S                 	7           /*!<TI1 Selection */
#define  OIS1                   	8           /*!<Output Idle state 1 (OC1 output) */
#define  OIS1N              	9            /*!<Output Idle state 1 (OC1N output) */
#define  OIS2                 	10           /*!<Output Idle state 2 (OC2 output) */
#define  OIS2N                  	11           /*!<Output Idle state 2 (OC2N output) */
#define  OIS3                   	12            /*!<Output Idle state 3 (OC3 output) */
#define  OIS3N                 	13            /*!<Output Idle state 3 (OC3N output) */
#define  OIS4                  	14           /*!<Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  SMS                     	0            /*!<SMS[2:0] bits (Slave mode selection) */

#define  TS                       	4            /*!<TS[2:0] bits (Trigger selection) */

#define  MSM                   	7            /*!<Master/slave mode */

#define  ETF                      	8           /*!<ETF[3:0] bits (External trigger filter) */

#define  ETPS                  	12           /*!<ETPS[1:0] bits (External trigger prescaler) */

#define  ECE                      	14            /*!<External clock enable */
#define  ETP                     	15           /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  UIE                     	0           /*!<Update interrupt enable */
#define  CC1IE                  	1           /*!<Capture/Compare 1 interrupt enable */
#define  CC2IE                  	2           /*!<Capture/Compare 2 interrupt enable */
#define  CC3IE                 	3           /*!<Capture/Compare 3 interrupt enable */
#define  CC4IE                 	4           /*!<Capture/Compare 4 interrupt enable */
#define  COMIE                  	5           /*!<COM interrupt enable */
#define  TIE                     	6            /*!<Trigger interrupt enable */
#define  BIE                      	7           /*!<Break interrupt enable */
#define  UDE                     	8           /*!<Update DMA request enable */
#define  CC1DE                	9            /*!<Capture/Compare 1 DMA request enable */
#define  CC2DE                 	10           /*!<Capture/Compare 2 DMA request enable */
#define  CC3DE                 	11           /*!<Capture/Compare 3 DMA request enable */
#define  CC4DE                 	12           /*!<Capture/Compare 4 DMA request enable */
#define  COMDE                	13           /*!<COM DMA request enable */
#define  TDE                     	14           /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define  UIF                     	0           /*!<Update interrupt Flag */
#define  CC1IF                  	1           /*!<Capture/Compare 1 interrupt Flag */
#define  CC2IF                 	2            /*!<Capture/Compare 2 interrupt Flag */
#define  CC3IF                  	3           /*!<Capture/Compare 3 interrupt Flag */
#define  CC4IF                  	4           /*!<Capture/Compare 4 interrupt Flag */
#define  COMIF                	5           /*!<COM interrupt Flag */
#define  TIF                     	6           /*!<Trigger interrupt Flag */
#define  BIF                      	7          /*!<Break interrupt Flag */
#define  CC1OF                 	9           /*!<Capture/Compare 1 Overcapture Flag */
#define  CC2OF                 	10           /*!<Capture/Compare 2 Overcapture Flag */
#define  CC3OF                 	11         /*!<Capture/Compare 3 Overcapture Flag */
#define  CC4OF                 	12           /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  UG                       	0              /*!<Update Generation */
#define  CC1G                    	1               /*!<Capture/Compare 1 Generation */
#define  CC2G                   	2              /*!<Capture/Compare 2 Generation */
#define  CC3G                    	3               /*!<Capture/Compare 3 Generation */
#define  CC4G                  	4             /*!<Capture/Compare 4 Generation */
#define  COMG                  	5             /*!<Capture/Compare Control Update Generation */
#define  TG                       	6             /*!<Trigger Generation */
#define  BG                        	7              /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  CC1S                  	0           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */

#define  OC1FE                 	2           /*!<Output Compare 1 Fast enable */
#define  OC1PE                  	3           /*!<Output Compare 1 Preload enable */

#define  OC1M                   	4           /*!<OC1M[2:0] bits (Output Compare 1 Mode) */

#define  OC1CE                  	7           /*!<Output Compare 1Clear Enable */

#define  CC2S                   	8           /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */

#define  OC2FE                	10           /*!<Output Compare 2 Fast enable */
#define  OC2PE                  	11           /*!<Output Compare 2 Preload enable */

#define  OC2M                   	12            /*!<OC2M[2:0] bits (Output Compare 2 Mode) */

#define  OC2CE                 	15           /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  IC1PSC               	2           /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */

#define  IC1F                    	4           /*!<IC1F[3:0] bits (Input Capture 1 Filter) */

#define  IC2PSC                	10          /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */

#define  IC2F                   	12           /*!<IC2F[3:0] bits (Input Capture 2 Filter) */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  CC3S                    	0           /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */

#define  OC3FE                 	2           /*!<Output Compare 3 Fast enable */
#define  OC3PE                 	3          /*!<Output Compare 3 Preload enable */

#define  OC3M                  	4           /*!<OC3M[2:0] bits (Output Compare 3 Mode) */

#define  OC3CE                  	7           /*!<Output Compare 3 Clear Enable */

#define  CC4S                    	8           /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */

#define  OC4FE                  	10           /*!<Output Compare 4 Fast enable */
#define  OC4PE                 	11           /*!<Output Compare 4 Preload enable */

#define  OC4M                   	12           /*!<OC4M[2:0] bits (Output Compare 4 Mode) */

#define  OC4CE                  	15           /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  IC3PSC               	2           /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */

#define  IC3F                   	4          /*!<IC3F[3:0] bits (Input Capture 3 Filter) */

#define  IC4PSC                	10           /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */

#define  IC4F                     	12           /*!<IC4F[3:0] bits (Input Capture 4 Filter) */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  CC1E                     0         /*!<Capture/Compare 1 output enable */
#define  CC1P                   	1          /*!<Capture/Compare 1 output Polarity */
#define  CC1NE                 	2           /*!<Capture/Compare 1 Complementary output enable */
#define  CC1NP                  	3           /*!<Capture/Compare 1 Complementary output Polarity */
#define  CC2E                   	4           /*!<Capture/Compare 2 output enable */
#define  CC2P                   	5            /*!<Capture/Compare 2 output Polarity */
#define  CC2NE                	6            /*!<Capture/Compare 2 Complementary output enable */
#define  CC2NP                 	7           /*!<Capture/Compare 2 Complementary output Polarity */
#define  CC3E                    	8          /*!<Capture/Compare 3 output enable */
#define  CC3P                   	9           /*!<Capture/Compare 3 output Polarity */
#define  CC3NE                	10          /*!<Capture/Compare 3 Complementary output enable */
#define  CC3NP                  	11           /*!<Capture/Compare 3 Complementary output Polarity */
#define  CC4E                   	12          /*!<Capture/Compare 4 output enable */
#define  CC4P                    	13           /*!<Capture/Compare 4 output Polarity */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  DTG                    	0           /*!<DTG[0:7] bits (Dead-Time Generator set-up) */

#define  LOCK                    	8           /*!<LOCK[1:0] bits (Lock Configuration) */

#define  OSSI                   	10           /*!<Off-State Selection for Idle mode */
#define  OSSR                   	11          /*!<Off-State Selection for Run mode */
#define  BKE                      	12          /*!<Break enable */
//#define  BKP                      	13          /*!<Break Polarity */
#define  AOE                     	14           /*!<Automatic Output enable */
#define  MOE                    	15           /*!<Main Output enable */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  DBA                    	0           /*!<DBA[4:0] bits (DMA Base Address) */

#define  DBL                       8            /*!<DBL[4:0] bits (DMA Burst Length) */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  DMAB                   	0          /*!<DMA register for burst accesses */

/******************************************************************************/
/*                                                                            */
/*                               SystemTick                                   */
/*                                                                            */
/******************************************************************************/

/*****************  Bit definition for SysTick_CTRL register  *****************/
#define  SysTick_ENABLE   		0	//      ((uint32_t)0x00000001)        /*!< Counter enable */
#define  SysTick_TICKINT            	1	//    ((uint32_t)0x00000002)        /*!< Counting down to 0 pends the SysTick handler */
#define  SysTick_CLKSOURCE      	2	//        ((uint32_t)0x00000004)        /*!< Clock source */
#define  SysTick_COUNTFLAG       	16	//       ((uint32_t)0x00010000)        /*!< Count Flag */

/*****************  Bit definition for SysTick_LOAD register  *****************/
#define  SysTick_RELOAD              	0	//   ((uint32_t)0x00FFFFFF)        /*!< Value to load into the SysTick Current Value Register when the counter reaches 0 */

/*****************  Bit definition for SysTick_VAL register  ******************/
#define  SysTick_CURRENT           	0	//      ((uint32_t)0x00FFFFFF)        /*!< Current value at the time the register is accessed */

/*****************  Bit definition for SysTick_CALIB register  ****************/
#define  SysTick_TENMS               	0	//  ((uint32_t)0x00FFFFFF)        /*!< Reload value to use for 10ms timing */
#define  SysTick_SKEW               	30	//   ((uint32_t)0x40000000)        /*!< Calibration value is not exactly 10 ms */
#define  SysTick_NOREF                	31	// ((uint32_t)0x80000000)        /*!< The reference clock is not provided */

// GPIO port configuration constants
#define	GPIO_Mode_In				0x00
#define	GPIO_Mode_Out10M			0x01
#define	GPIO_Mode_Out2M			0x02
#define	GPIO_Mode_Out50M			0x03

#define	GPIO_CNF_GP_PP			0x00
#define	GPIO_CNF_GP_OD			0x04
#define	GPIO_CNF_AF_PP			0x08
#define	GPIO_CNF_AF_OD			0x0C
#define	GPIO_CNF_AnalogIn			0x00
#define	GPIO_CNF_Floating			0x04
#define	GPIO_CNF_IPD				0x08
#define	GPIO_CNF_IPU				0x08

/******************  Bit definition for FSMC_BCR registers  *******************/
#define  CBURSTRW           	16        /*!<Write burst enable */
#define  EXTMOD               	14        /*!<Extended mode enable */
#define  WAITEN               	13        /*!<Wait enable bit */
#define  WREN                   	12        /*!<Write enable bit */
#define  WAITCFG             	11        /*!<Wait timing configuration */
#define  WRAPMOD            	10        /*!<Wrapped burst mode support */
#define  WAITPOL              9        /*!<Wait signal polarity bit */
#define  BURSTEN             	8        /*!<Burst enable bit */
#define  FACCEN             	6        /*!<Flash access enable */
#define  MWID                   	4        /*!<MWID[1:0] bits (Memory data bus width) */
#define  MTYP                    	2        /*!<MTYP[1:0] bits (Memory type) */
#define  MUXEN                 	1        /*!<Address/data multiplexing enable bit */
#define  MBKEN            	0        /*!<Memory bank enable bit */

/******************  Bit definition for FSMC_BTR and FSMC_BWTR registers  ******************/
#define  ACCMOD               	28        /*!<ACCMOD[1:0] bits (Access mode) */
#define  DATLAT                	24       /*!<DATLA[3:0] bits (Data latency) */
#define  CLKDIV                 	20        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  BUSTURN             	16        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  DATAST               	8        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  ADDHLD                4        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  ADDSET           	0       /*!<ADDSET[3:0] bits (Address setup phase duration) */

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define  AWD             		0               /*!<Analog watchdog flag */
#define  EOC                      1               /*!<End of conversion */
#define  JEOC                     2               /*!<Injected channel end of conversion */
#define  JSTRT                   3               /*!<Injected channel Start flag */
#define  STRT                     4               /*!<Regular channel Start flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  AWDCH                  0            /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  EOCIE                   	5           /*!<Interrupt enable for EOC */
#define  AWDIE                 	6              /*!<AAnalog Watchdog interrupt enable */
#define  JEOCIE                  7           /*!<Interrupt enable for injected channels */
#define  SCAN                    8           /*!<Scan mode */
#define  AWDSGL               9             /*!<Enable the watchdog on a single channel in scan mode */
#define  JAUTO                   10            /*!<Automatic injected group conversion */
#define  DISCEN                  11            /*!<Discontinuous mode on regular channels */
#define  JDISCEN                12             /*!<Discontinuous mode on injected channels */
#define  DISCNUM               13           /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  DUALMOD              16              /*!<DUALMOD[3:0] bits (Dual mode selection) */
#define  JAWDEN                22             /*!<Analog watchdog enable on injected channels */
#define  AWDEN                  23            /*!<Analog watchdog enable on regular channels */

  
/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADON           0        //     ((uint32_t)0x00000001)        /*!<A/D Converter ON / OFF */
#define  CONT            1        //    ((uint32_t)0x00000002)        /*!<Continuous Conversion */
#define  CAL               2     //     ((uint32_t)0x00000004)        /*!<A/D Calibration */
#define  RSTCAL        3       //       ((uint32_t)0x00000008)        /*!<Reset Calibration */
#define  DMA              8     //      ((uint32_t)0x00000100)        /*!<Direct Memory access mode */
#define  ALIGN           11    //        ((uint32_t)0x00000800)        /*!<Data Alignment */
#define  JEXTSEL        12	//             ((uint32_t)0x00007000)        /*!<JEXTSEL[2:0] bits (External event select for injected group) */
#define  JEXTTRIG      15	//              ((uint32_t)0x00008000)        /*!<External Trigger Conversion mode for injected channels */
#define  EXTSEL         17     //        ((uint32_t)0x000E0000)        /*!<EXTSEL[2:0] bits (External Event Select for regular group) */
#define  EXTTRIG       20	//              ((uint32_t)0x00100000)        /*!<External Trigger Conversion mode for regular channels */
#define  JSWSTART     21	//               ((uint32_t)0x00200000)        /*!<Start Conversion of injected channels */
#define  SWSTART      22	//               ((uint32_t)0x00400000)        /*!<Start Conversion of regular channels */
#define  TSVREFE       23	//              ((uint32_t)0x00800000)        /*!<Temperature Sensor and VREFINT Enable */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  SMP10            0 	//        ((uint32_t)0x00000007)        /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  SMP11            3 	//       ((uint32_t)0x00000038)        /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  SMP12            6 	//        ((uint32_t)0x000001C0)        /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  SMP13            9	//         ((uint32_t)0x00000E00)        /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  SMP14            12	//         ((uint32_t)0x00007000)        /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  SMP15            15	//       ((uint32_t)0x00038000)        /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  SMP16            18 	//        ((uint32_t)0x001C0000)        /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  SMP17            21 	//        ((uint32_t)0x00E00000)        /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  SMP0             0  	//        ((uint32_t)0x00000007)        /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  SMP1             3   	//      ((uint32_t)0x00000038)        /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  SMP2             6   	//       ((uint32_t)0x000001C0)        /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  SMP3             9  	//       ((uint32_t)0x00000E00)        /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  SMP4             12  	//       ((uint32_t)0x00007000)        /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  SMP5             15   	//       ((uint32_t)0x00038000)        /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  SMP6             18   	//       ((uint32_t)0x001C0000)        /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  SMP7             21  	//        ((uint32_t)0x00E00000)        /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  SMP8             24  	//        ((uint32_t)0x07000000)        /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  SMP9             27  	//       ((uint32_t)0x38000000)        /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  ((uint16_t)0x0FFF)            /*!<Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  ((uint16_t)0x0FFF)            /*!<Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  ((uint16_t)0x0FFF)            /*!<Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  ((uint16_t)0x0FFF)            /*!<Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          ((uint16_t)0x0FFF)            /*!<Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          ((uint16_t)0x0FFF)            /*!<Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  SQ13        0      	//           ((uint32_t)0x0000001F)        /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
#define  SQ14        5      	//          ((uint32_t)0x000003E0)        /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
#define  SQ15        10    	//           ((uint32_t)0x00007C00)        /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
#define  SQ16        15    	//             ((uint32_t)0x000F8000)        /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
#define  L               20     	//        ((uint32_t)0x00F00000)        /*!<L[3:0] bits (Regular channel sequence length) */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  SQ7         0      	//           ((uint32_t)0x0000001F)        /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
#define  SQ8         5       	//          ((uint32_t)0x000003E0)        /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
#define  SQ9         10     	//            ((uint32_t)0x00007C00)        /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
#define  SQ10       15      	//            ((uint32_t)0x000F8000)        /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
#define  SQ11       20      	//            ((uint32_t)0x01F00000)        /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
#define  SQ12       25      	//            ((uint32_t)0x3E000000)        /*!<SQ12[4:0] bits (12th conversion in regular sequence) */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  SQ1        0        	//          ((uint32_t)0x0000001F)        /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
#define  SQ2        5        	//          ((uint32_t)0x000003E0)        /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  SQ3        10      	//            ((uint32_t)0x00007C00)        /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  SQ4        15      	//            ((uint32_t)0x000F8000)        /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
#define  SQ5        20      	//            ((uint32_t)0x01F00000)        /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
#define  SQ6        25       	//           ((uint32_t)0x3E000000)        /*!<SQ6[4:0] bits (6th conversion in regular sequence) */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define  JSQ1      0         	//          ((uint32_t)0x0000001F)        /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */  
#define  JSQ2      5       	//            ((uint32_t)0x000003E0)        /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  JSQ3      10       	//            ((uint32_t)0x00007C00)        /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  JSQ4      15       	//            ((uint32_t)0x000F8000)        /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  JL           20        	//        ((uint32_t)0x00300000)        /*!<JL[1:0] bits (Injected Sequence length) */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      ((uint16_t)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      ((uint16_t)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      ((uint16_t)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      ((uint16_t)0xFFFF)            /*!<Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         ((uint32_t)0x0000FFFF)        /*!<Regular data */
#define  ADC_DR_ADC2DATA                     ((uint32_t)0xFFFF0000)        /*!<ADC2 data */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define  CGIF1              0      	//       /*!< Channel 1 Global interrupt clearr */
#define  CTCIF1            1       	//       /*!< Channel 1 Transfer Complete clear */
#define  CHTIF1            2        	//        /*!< Channel 1 Half Transfer clear */
#define  CTEIF1            3         	//       /*!< Channel 1 Transfer Error clear */
#define  CGIF2              4         	//        /*!< Channel 2 Global interrupt clear */
#define  CTCIF2            5         	//        /*!< Channel 2 Transfer Complete clear */
#define  CHTIF2            6       	//        /*!< Channel 2 Half Transfer clear */
#define  CTEIF2            7         	//       /*!< Channel 2 Transfer Error clear */
#define  CGIF3             8          	//        /*!< Channel 3 Global interrupt clear */
#define  CTCIF3            9          //        /*!< Channel 3 Transfer Complete clear */
#define  CHTIF3            10        //        /*!< Channel 3 Half Transfer clear */
#define  CTEIF3            11        //        /*!< Channel 3 Transfer Error clear */
#define  CGIF4              12       //        /*!< Channel 4 Global interrupt clear */
#define  CTCIF4            13       //       /*!< Channel 4 Transfer Complete clear */
#define  CHTIF4            14       //       /*!< Channel 4 Half Transfer clear */
#define  CTEIF4            15       //        /*!< Channel 4 Transfer Error clear */
#define  CGIF5              16       //       /*!< Channel 5 Global interrupt clear */
#define  CTCIF5            17       //        /*!< Channel 5 Transfer Complete clear */
#define  CHTIF5            18       //        /*!< Channel 5 Half Transfer clear */
#define  CTEIF5             19      //        /*!< Channel 5 Transfer Error clear */
#define  CGIF6              20       //        /*!< Channel 6 Global interrupt clear */
#define  CTCIF6            21       //        /*!< Channel 6 Transfer Complete clear */
#define  CHTIF6            22       //        /*!< Channel 6 Half Transfer clear */
#define  CTEIF6            23       //       /*!< Channel 6 Transfer Error clear */
#define  CGIF7              24       //        /*!< Channel 7 Global interrupt clear */
#define  CTCIF7            25       //        /*!< Channel 7 Transfer Complete clear */
#define  CHTIF7            26       //        /*!< Channel 7 Half Transfer clear */
#define  CTEIF7             27      //        /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCRx register  *******************/
#define  EN               	0        	//         /*!< Channel enable*/
#define  TCIE            	1        	//            /*!< Transfer complete interrupt enable */
#define  HTIE           	2        	//           /*!< Half Transfer interrupt enable */
#define  TEIE           	3         	//            /*!< Transfer error interrupt enable */
#define  DIR             	4         	//           /*!< Data transfer direction */
							//				0: Read from peripheral
							//				1: Read from memory
#define  CIRC           	5        	//            /*!< Circular mode */
							//				0: Circular mode disabled
							//				1: Circular mode enabled
#define  PINC           	6        	//           /*!< Peripheral increment mode */
							//				0: Peripheral increment mode disabled
							//				1: Peripheral increment mode enabled
#define  MINC          	7        	//          /*!< Memory increment mode */
							//				0: Memory increment mode disabled
							//				1: Memory increment mode enabled
#define  PSIZE         	8        	//           /*!< PSIZE[1:0] bits (Peripheral size) */
							//				00: 8-bits
							//				01: 16-bits
							//				10: 32-bits
							//				11: Reserved
#define  MSIZE       	10          //           /*!< MSIZE[1:0] bits (Memory size) */
							//				00: 8-bits
							//				01: 16-bits
							//				10: 32-bits
							//				11: Reserved
#define  PL             	12          //           /*!< PL[1:0] bits(Channel Priority level) */
							//				00: Low
							//				01: Medium
							//				10: High
							//				11: Very high
#define  MEM2MEM   	14          //           /*!< Memory to memory mode */

/******************************************************************************/
/*                                                                            */
/*                       Flash Controller                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for Flash_SR register  ********************/
// Bit 5 EOP: End of operation
//	Set by hardware when a Flash operation (programming / erase) is completed. Reset by 
//	writing a 1
//	Note: EOP is asserted at the end of each successful program or erase operation
// Bit 4 WRPRTERR: Write protection error
//	Set by hardware when programming a write-protected address of the Flash memory.
//	Reset by writing 1.
// Bit 3 Reserved, must be kept cleared.
// Bit 2 PGERR: Programming error
//	Set by hardware when an address to be programmed contains a value different from 
//	'0xFFFF' before programming.
//	Reset by writing 1.
//	Note: The STRT bit in the FLASH_CR register should be reset before starting a programming 
//	operation.
// Bit 1 Reserved, must be kept cleared
// Bit 0 BSY: Busy
//	This indicates that a Flash operation is in progress. This is set on the beginning of a Flash 
//	operation and reset when the operation finishes or when an error occurs.

#define	Flash_SR_EOP			5
#define	Flash_SR_WRPRTERR		4
//#define	Flash_SR_Res			3
#define	Flash_SR_PGERR			2
//#define	Flash_SR_Res			1
#define	Flash_SR_BSY			0

/********************  Bit definition for Flash_CR register  ********************/
// Bits 31:13 Reserved, must be kept cleared.
// Bit 12 EOPIE: End of operation interrupt enable
//	This bit enables the interrupt generation when the EOP bit in the FLASH_SR register goes 
//	to 1.
//	0: Interrupt generation disabled
//	1: Interrupt generation enabled
// Bit 11 Reserved, must be kept cleared
// Bit 10 ERRIE: Error interrupt enable
//	This bit enables the interrupt generation on an FPEC error (when PGERR / WRPRTERR are 
//	set in the FLASH_SR register).
//	0: Interrupt generation disabled
//	1: Interrupt generation enabled
// Bit 9 OPTWRE: Option bytes write enable
//	When set, the option bytes can be programmed. This bit is set on writing the correct key 
//	sequence to the FLASH_OPTKEYR register.
//	This bit can be reset by software
// Bit 8 Reserved, must be kept cleared. 
// Bit 7 LOCK: Lock
//	Write to 1 only. When it is set, it indicates that the FPEC and FLASH_CR are locked. This bit 
//	is reset by hardware after detecting the unlock sequence.
//	In the event of unsuccessful unlock operation, this bit remains set until the next reset.
// Bit 6 STRT: Start
//	This bit triggers an ERASE operation when set. This bit is set only by software and reset 
//	when the BSY bit is reset.
// Bit 5 OPTER: Option byte erase
//	Option byte erase chosen.
// Bit 4 OPTPG: Option byte programming
//	Option byte programming chosen.
// Bit 3 Reserved, must be kept cleared.
// Bit 2 MER: Mass erase
//	Erase of all user pages chosen.
// Bit 1 PER: Page erase
//	Page Erase chosen.
// Bit 0 PG: Programming
//	Flash programming chosen.

#define	Flash_CR_EOPIE			12
#define	Flash_CR_ERRIE			10
#define	Flash_CR_OPTWRE		9
#define	Flash_CR_LOCK			7
#define	Flash_CR_STRT			6
#define	Flash_CR_OPTER			5
#define	Flash_CR_OPTPG			4
#define	Flash_CR_MER			2
#define	Flash_CR_PER			1
#define	Flash_CR_PG			0

/********************  Bit definition for FLASH_OBR register  ********************/

// Bits 31:26 Reserved, must be kept cleared.
// Bits 25:18 Data1
// Bits 17:10 Data0
// Bits 9:2 USER: User option bytes
//	This contains the user option byte loaded by the OBL.
//	Bits [9:5]: Not used (if these bits are written in the Flash option byte, they will be read in this 
//	register with no effect on the device.)
// Bit 4: nRST_STDBY
// Bit 3: nRST_STOP
// Bit 2: WDG_SW
// Bit 1 RDPRT: Read protection
//	When set, this indicates that the Flash memory is read-protected.
//	Note: This bit is read-only. 
// Bit 0 OPTERR: Option byte error
//	When set, this indicates that the loaded option byte and its complement do not match. The 
//	corresponding byte and its complement are read as 0xFF in the FLASH_OBR or 
//	FLASH_WRPR register.
//	Note: This bit is read-only.

#define	Flash_OBR_Data1			18
#define	Flash_OBR_Data0			10
#define	Flash_OBR_USER				2
#define	Flash_OBR_nRST_STDBY		4
#define	Flash_OBR_nRST_STOP		3
#define	Flash_OBR_nRST_SW			2
#define	Flash_OBR_RDPRT			1
#define	Flash_OBR_OPTERR			0

#define	Flash_KEY1 					0x45670123
#define	Flash_KEY2 					0xCDEF89AB



#define	I2C_QuadCycle		10

extern	U16		GTimer;
extern	U8		GTimeout;

extern	U16		TimerKeyScan;
extern	U16		TFT_Controller;

// ====================================================
//	Macros
//

// ====================================================
//	Function Prototype Declarations
//
void	Clock_Init(void);
void Port_Init(void);
void	USART1_Init(void);
void	UartPutc(U8 ch, USART_TypeDef* USARTx);
void	uputs(U8 *s, USART_TypeDef* USARTx);
void	ADC2_Init(void);
U16	ADC_Poll(ADC_TypeDef * adc, U8 chn);
void	TIM3_Init(void);
void	TIM4_Init(void);
void	SysTick_Init(void);

void	TFT_Init_Ili9325(void);
void	TFT_CmdWrite(U16 Reg, U16 Data);
void	TFT_AccessGRAM(void);
void	TFT_AccessGRAM_End(void);
U16 TFT_ReadID_Ili9325(void);

void	TFT_Init_Ili9341(void);
void	write_comm(U8 commport);
void write_data(U8 data);
U32	TFT_ReadID_Ili9341(void);

void assert_failed(U8 * file, U32 line);
void NVIC_Configuration(void);
void	OutputVSen(void);

void	I2C_Start(void);
void	I2C_Stop(void);
void	I2C_SendByte(U8 byte);
U8	I2C_RecvByte(void);
U8	I2C_CheckAck(void);
void	I2C_Ack(void);
void	I2C_Nak(void);
void	I2C_ReSync(void);
void	SetSDA_In(void);
void SetSDA_Out(void);

#endif	// Board_h


