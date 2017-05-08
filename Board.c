//////////////////////////////////////////////////////////////////////////////
//
// 	Filename:	Board.c
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
//-----------------------------------------------------------------------------
// 	Includes
//-----------------------------------------------------------------------------

#include "stm32f10x.h"

#include "Common.h"
#include "Board.h"
#include	"libdso150.h"

// ===========================================================
//	File Scope Variables
// ===========================================================
//
const	U8	VSenCtrlTab[13] = {
	// {X, X, X, SENSEL[3:0], X }
	0b00000110,		// 20V
	0b00001010,		// 10V
	0b00000000,		// 5V
	0b00001110,		// 2V
	0b00001100,		// 1V
	0b00001000,		// 0.5V
	0b00010110,		// 0.2V
	0b00011010,		// 0.1V
	0b00010000,		// 50mV
	0b00011110,		// 20mV
	0b00011100,		// 10mV
	0b00011000,		// 5mV
	0b00000010		// GND
	};


U16		GTimer;
U8		GTimeout;

U16		TimerKeyScan;

U16		TFT_Controller;

// ===========================================================
//	Function Definitions
// ===========================================================

//-----------------------------------------------------------------------------
// Clock_Init
//-----------------------------------------------------------------------------
//
void	Clock_Init(void)
{
 RCC->CR =  (1 << HSION)  		/*!< Internal High Speed clock enable */
			|(0 << HSIRDY)     	/*!< Internal High Speed clock ready flag */
			|(0x10 << HSITRIM)     /*!< Internal High Speed clock trimming */
			|(0 << HSICAL)     	/*!< Internal High Speed clock Calibration */
			|(1 << HSEON)     		/*!< External High Speed clock enable */
			|(0 << HSERDY)     	/*!< External High Speed clock ready flag */
			|(0 << HSEBYP)     	/*!< External High Speed clock Bypass */
			|(0 << CSSON)     	/*!< Clock Security System enable */
			|(0 << PLLON)     		/*!< PLL enable */
			|(0 << PLLRDY);    	/*!< PLL clock ready flag */

//	MCO[2:0] : Microcontroller clock output
//		0xx: No clock
//		100: System clock (SYSCLK) selected
//		101: HSI clock selected
//		110: HSE clock selected
//		111: PLL clock divided by 2 selected
//		
//	USBPRE: USB prescaler
//	Set and cleared by software to generate 48 MHz USB clock. This bit must be valid before
//	enabling the USB clock in the RCC_APB1ENR register. This bit can’t be reset if the USB
//	clock is enabled.
//		0: PLL clock is divided by 1.5
//		1: PLL clock is not divided
//		
//	PLLMUL[3:0] : PLL multiplication factor
//	These bits are written by software to define the PLL multiplication factor. These bits can be
//	written only when PLL is disabled.
//		0000: PLL input clock x 2
//		0001: PLL input clock x 3
//		0010: PLL input clock x 4
//		0011: PLL input clock x 5
//		0100: PLL input clock x 6
//		0101: PLL input clock x 7
//		0110: PLL input clock x 8
//		0111: PLL input clock x 9
//		1000: PLL input clock x 10
//		1001: PLL input clock x 11
//		1010: PLL input clock x 12
//		1011: PLL input clock x 13
//		1100: PLL input clock x 14
//		1101: PLL input clock x 15
//		1110: PLL input clock x 16
//		1111: PLL input clock x 16
//
//	PLLXTPRE: HSE divider for PLL entry
//	Set and cleared by software to divide HSE before PLL entry. This bit can be written only
//	when PLL is disabled.
//		0: HSE clock not divided
//		1: HSE clock divided by 2
//		
//	PLLSRC: PLL entry clock source
//	Set and cleared by software to select PLL clock source. This bit can be written only when
//	PLL is disabled.
//		0: HSI oscillator clock / 2 selected as PLL input clock
//		1: HSE oscillator clock selected as PLL input clock	
//		
//	ADCPRE[1:0] : ADC prescaler
//	Set and cleared by software to select the frequency of the clock to the ADCs.
//		00: PLCK2 divided by 2
//		01: PLCK2 divided by 4
//		10: PLCK2 divided by 6
//		11: PLCK2 divided by 8
//	
//	PPRE2[2:0] : APB high-speed prescaler (APB2)
//	Set and cleared by software to control the division factor of the APB high-speed clock
//	(PCLK2).
//		0xx: HCLK not divided
//		100: HCLK divided by 2
//		101: HCLK divided by 4
//		110: HCLK divided by 8
//		111: HCLK divided by 16

//	PPRE1[2:0] : APB low-speed prescaler (APB1)
//	Set and cleared by software to control the division factor of the APB low-speed clock
//	(PCLK1).
//	Warning: the software has to set correctly these bits to not exceed 36 MHz on this domain.
//		0xx: HCLK not divided
//		100: HCLK divided by 2
//		101: HCLK divided by 4
//		110: HCLK divided by 8
//		111: HCLK divided by 16

//	HPRE[3:0] : AHB prescaler
//	Set and cleared by software to control the division factor of the AHB clock.
//		0xxx: SYSCLK not divided
//		1000: SYSCLK divided by 2
//		1001: SYSCLK divided by 4
//		1010: SYSCLK divided by 8
//		1011: SYSCLK divided by 16
//		1100: SYSCLK divided by 64
//		1101: SYSCLK divided by 128
//		1110: SYSCLK divided by 256
//		1111: SYSCLK divided by 512	
//		
//	SWS[1:0] : System clock switch status
//	Set and cleared by hardware to indicate which clock source is used as system clock.
//		00: HSI oscillator used as system clock
//		01: HSE oscillator used as system clock
//		10: PLL used as system clock
//		11: not applicable

//	SW[1:0] : System clock switch
//	Set and cleared by software to select SYSCLK source.
//	Set by hardware to force HSI selection when leaving Stop and Standby mode or in case of
//	failure of the HSE oscillator used directly or indirectly as system clock (if the Clock Security
//	System is enabled).
//		00: HSI selected as system clock
//		01: HSE selected as system clock
//		10: PLL selected as system clock
//		11: not allowed
//
 RCC->CFGR = (0 << SW)     		/*!< SW[1:0] bits (System clock Switch) */
			|(0 << SWS)     		/*!< SWS[1:0] bits (System Clock Switch Status) */
			|(0 << HPRE)     		/*!< HPRE[3:0] bits (AHB prescaler)  [HCLK] */ 
			|(0b100 << PPRE1)     	/*!< PRE1[2:0] bits (APB1 prescaler) [PCLK1] */
			|(0 << PPRE2)     		/*!< PRE2[2:0] bits (APB2 prescaler) [PCLK2] */
			|(2 << ADCPRE)     	/*!< ADCPRE[1:0] bits (ADC prescaler) */
			|(1 << PLLSRC)     	/*!< PLL entry clock source */
			|(0 << PLLXTPRE)     	/*!< HSE divider for PLL entry */
			|(7 << PLLMULL)     	/*!< PLLMUL[3:0] bits (PLL multiplication factor) */
			|(0 << USBPRE)     	/*!< USB Device prescaler */
			|(0 << MCO);     		/*!< MCO[2:0] bits (Microcontroller Clock Output) */


 RCC->CIR = (0 << LSIRDYF)     	/*!< LSI Ready Interrupt flag */
			|(0 << LSERDYF)     	/*!< LSE Ready Interrupt flag */
			|(0 << HSIRDYF)     	/*!< HSI Ready Interrupt flag */
			|(0 << HSERDYF)     	/*!< HSE Ready Interrupt flag */
			|(0 << PLLRDYF)     	/*!< PLL Ready Interrupt flag */
			|(0 << CSSF)     		/*!< Clock Security System Interrupt flag */
			|(0 << LSIRDYIE )     	/*!< LSI Ready Interrupt Enable */
			|(0 << LSERDYIE)     	/*!< LSE Ready Interrupt Enable */
			|(0 << HSIRDYIE)     	/*!< HSI Ready Interrupt Enable */
			|(0 << HSERDYIE)     	/*!< HSE Ready Interrupt Enable */
			|(0 << PLLRDYIE)     	/*!< PLL Ready Interrupt Enable */
			|(0 << LSIRDYC)     	/*!< LSI Ready Interrupt Clear */
			|(0 << LSERDYC)     	/*!< LSE Ready Interrupt Clear */
			|(0 << HSIRDYC)     	/*!< HSI Ready Interrupt Clear */
			|(0 << HSERDYC)     	/*!< HSE Ready Interrupt Clear */
			|(0 << PLLRDYC)     	/*!< PLL Ready Interrupt Clear */
			|(0 << CSSC);	     	/*!< Clock Security System Interrupt Clear */


 RCC->APB2RSTR = (0 << AFIORST) 	/*!< Alternate Function I/O reset */
			|(0 << IOPARST)     	/*!< I/O port A reset */
			|(0 << IOPBRST)     	/*!< I/O port B reset */
			|(0 << IOPCRST)     	/*!< I/O port C reset */
			|(0 << IOPDRST)     	/*!< I/O port D reset */
			|(0 << IOPERST)     	/*!< I/O port E reset */
			|(0 << IOPFRST)     	/*!< I/O port F reset */
			|(0 << IOPGRST)     	/*!< I/O port G reset */
			|(0 << ADC1RST)     	/*!< ADC 1 interface reset */
			|(0 << ADC2RST)     	/*!< ADC 2 interface reset */
			|(0 << TIM1RST)     	/*!< TIM1 Timer reset */
			|(0 << SPI1RST)     	/*!< SPI 1 reset */
			|(0 << TIM8RST)     	/*!< TIM8 Timer reset */
			|(0 << USART1RST)     	/*!< USART1 reset */
			|(0 << ADC3RST);     	/*!< ADC3 interface reset */

 RCC->APB1RSTR = (0 << TIM2RST) /*!< Timer 2 reset */
			|(0 << TIM3RST)     	/*!< Timer 3 reset */
			|(0 << TIM4RST)     	/*!< Timer 4 reset */
			|(0 << TIM5RST)     	/*!< Timer 5 reset */
			|(0 << TIM6RST)     	/*!< Timer 6 reset */
			|(0 << TIM7RST)     	/*!< Timer 7 reset */
			|(0 << WWDGRST)     	/*!< Window Watchdog reset */
			|(0 << SPI2RST)     	/*!< SPI 2 reset */
			|(0 << SPI3RST)     	/*!< SPI 3 reset */
			|(0 << USART2RST)     	/*!< USART 2 reset */
			|(0 << USART3RST)     	/*!< RUSART 3 reset */
			|(0 << UART4RST )     	/*!< UART 4 reset */
			|(0 << UART5RST)     	/*!< UART 5 reset */
			|(0 << I2C1RST)     	/*!< I2C 1 reset */
			|(0 << I2C2RST)     	/*!< I2C 2 reset */
			|(0 << USBRST)     	/*!< USB Device reset */
			|(0 << CAN1RST)     	/*!< CAN1 reset */
			|(0 << BKPRST)     	/*!< Backup interface reset */
			|(0 << PWRRST)     	/*!< Power interface reset */
			|(0 << DACRST);     	/*!< DAC interface reset */

 
 RCC->AHBENR = (0 << SDIOEN)
				|(0 << FSMCEN)
				|(0 << CRCEN)
				|(1 << FLITFEN)
				|(1 << SRAMEN)
				|(0 << DMA2EN)
				|(1 << DMA1EN);

 RCC->APB1ENR = (0 << DACEN)
				|(0 << PWREN)
				|(0 << BKPEN)
				|(0 << CANEN)
				|(0 << USBEN)
				|(0 << I2C2EN)
				|(0 << I2C1EN)
				|(0 << UART5EN)
				|(0 << UART4EN)
				|(0 << USART3EN)
				|(0 << USART2EN)
				|(0 << SPI3EN)
				|(0 << SPI2EN)
				|(0 << WWDGEN)
				|(0 << TIM7EN)
				|(0 << TIM6EN)
				|(0 << TIM5EN)
				|(1 << TIM4EN)
				|(1 << TIM3EN)
				|(1 << TIM2EN);

 RCC->APB2ENR = (0 << ADC3EN)
				|(1 << USART1EN)
				|(0 << TIM8EN)
				|(0 << SPI1EN)
				|(1 << TIM1EN)
				|(1 << ADC2EN)
				|(1 << ADC1EN)
				|(0 << IOPGEN)
				|(0 << IOPFEN)
				|(0 << IOPEEN)
				|(1 << IOPDEN)
				|(1 << IOPCEN)
				|(1 << IOPBEN)
				|(1 << IOPAEN)
				|(1 << AFIOEN);

 RCC->BDCR = 0x00000000;
 RCC->CSR = 0x00000000;

 // Switch to HSE if it is ready
 if(BitTest(RCC->CR, (1 << HSERDY))) {
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_HSE;

	// Set PLL source to HSE
	RCC->CFGR |= (1 << PLLSRC);
	
 	}
 else {
	// Use HSI as PLL source
	RCC->CFGR &= ~(1 << PLLSRC);

 	}
 
 // Turn on PLL
 RCC->CR =  (1 << HSION)  		/*!< Internal High Speed clock enable */
			|(0 << HSIRDY)     	/*!< Internal High Speed clock ready flag */
			|(0x10 << HSITRIM)     /*!< Internal High Speed clock trimming */
			|(0 << HSICAL)     	/*!< Internal High Speed clock Calibration */
			|(1 << HSEON)     		/*!< External High Speed clock enable */
			|(0 << HSERDY)     	/*!< External High Speed clock ready flag */
			|(0 << HSEBYP)     	/*!< External High Speed clock Bypass */
			|(0 << CSSON)     	/*!< Clock Security System enable */
			|(1 << PLLON)     		/*!< PLL enable */
			|(0 << PLLRDY);    	/*!< PLL clock ready flag */

 Delay(50000);

 // Switch to PLL if it is ready
 if(BitTest(RCC->CR, (1 << PLLRDY))) {
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
 	}

}

//-----------------------------------------------------------------------------
// Misc_Init
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
//
void Port_Init(void)
{
 // Remap to make PB3 & PB4 available
 AFIO->MAPR &= ~AFIO_MAPR_SWJ_CFG;
 AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;	 

 GPIOA->CRL = ((GPIO_CNF_AnalogIn | GPIO_Mode_In) << (0*4))		// ADC1_IN0
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (1*4))		// Output, SENSEL0
 			|((GPIO_CNF_GP_PP |GPIO_Mode_Out50M) << (2*4))		// Output, SENSEL1
 			|((GPIO_CNF_GP_PP |GPIO_Mode_Out50M) << (3*4))		// Output, SENSEL2
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (4*4))		// Output, SENSEL3
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (5*4))			// Input, CPLSEL
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (6*4))		// Output, TFT_nRD
 			|((GPIO_CNF_AF_PP | GPIO_Mode_Out50M) << (7*4));		// Output, Test signal


 GPIOA->CRH = ((GPIO_CNF_Floating| GPIO_Mode_In) << (8 - 8)*4)		
 			|((GPIO_CNF_AF_PP |GPIO_Mode_Out50M) << (9 - 8)*4)	// TX1
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << (10 - 8)*4)			// RX1
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (11 - 8)*4)
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (12 - 8)*4)
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (13 - 8)*4)	// SWDIO. 
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (14 - 8)*4)	// SWCLK. 
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (15 - 8)*4);	// LED 

 GPIOA->ODR = 0xFFFF;

 GPIOB->CRL = ((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (0*4))		// TFT port - D0, ENC_A
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (1*4))		// TFT port - D1, ENC_B
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (2*4))		// TFT port - D2
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (3*4))		// TFT port - D3, ENC_PB
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (4*4))		// TFT port - D4, SW2
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (5*4))		// TFT port - D5, SW3
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (6*4))		// TFT port - D6, SW4	
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (7*4));		// TFT port - D7, SW5		

 GPIOB->CRH = ((GPIO_CNF_Floating | GPIO_Mode_In) << ((8 - 8)*4))		
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << ((9 - 8)*4))		// Output, TFT_nRESET
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << ((10 - 8)*4))		// SCL
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << ((11 - 8)*4))		// SDA
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << ((12 - 8)*4))		// AMPSEL (for test signal)
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((13 - 8)*4))			// 
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((14 - 8)*4))			// 
 			|((GPIO_CNF_IPU | GPIO_Mode_In) << ((15 - 8)*4));			// 
 			
 GPIOB->ODR = 0xFFFF;

 GPIOC->CRH = ((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (13 - 8)*4)		// TFT_nCS
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (14 - 8)*4)		// TFT_RS
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (15 - 8)*4);		// TFT_nWR

 GPIOC->ODR = 0xFFFF;
 
 GPIOD->CRL = ((GPIO_CNF_Floating | GPIO_Mode_In) << (0*4))		
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (1*4));		
 
}

void	USART1_Init(void)
{
 USART_InitTypeDef USART_InitStructure;
 
  USART_InitStructure.USART_BaudRate = 38400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);
  
  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);

}

void	UartPutc(U8 ch, USART_TypeDef* USARTx)
{
 while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET) {
	}
 USART_SendData(USARTx, ch);
}

void	uputs(U8 *s, USART_TypeDef* USARTx)
{
 while(*s != 0) {
 	UartPutc(*s, USARTx);
	s++;
 	}
}

 void	TIM3_Init(void)
{
 // Disable counter first
 TIM3->CR1 = (0 << CEN)    	    	//!<Counter enable //
		| (0 << UDIS)           		//!<Update disable //
		| (0 << URS)          		//!<Update request source //
		| (0 << OPM)          		//!<One pulse mode //
		| (0 << DIR)          		//!<Direction. 0: Up, 1: Down
		| (0 << CMS)          		//!<CMS[1:0] bits (Center-aligned mode selection) //
		| (1 << ARPE)          		//!<Auto-reload preload enable //
		| (0 << CKD);          		//!<CKD[1:0] bits (clock division for filtering) 0 = 1/1, 1 = 1/2, 2 = 1/4

 TIM3->CR2 = (0 << CCPC)           	//<Capture/Compare Preloaded Control //
		| (0 << CCUS)           		//<Capture/Compare Control Update Selection //
		| (0 << CCDS)           		//<Capture/Compare DMA Selection //
		| (0 << MMS)           		//<MMS[2:0] bits (Master Mode Selection) //
		| (0 << TI1S)           		//<TI1 Selection //
		| (0 << OIS1)           		//<Output Idle state 1 (OC1 output) //
		| (0 << OIS1N)           		//<Output Idle state 1 (OC1N output) //
		| (0 << OIS2)           		//<Output Idle state 2 (OC2 output) //
		| (0 << OIS2N)           		//<Output Idle state 2 (OC2N output) //
		| (0 << OIS3)           		//<Output Idle state 3 (OC3 output) //
		| (0 << OIS3N)           		//<Output Idle state 3 (OC3N output) //
		| (0 << OIS4);           		//<Output Idle state 4 (OC4 output) //

 TIM3->SMCR = (0 << SMS)           	//<SMS[2:0] bits (Slave mode selection) //
		| (0 << TS)           		//<TS[2:0] bits (Trigger selection) //
		| (0 << MSM)           		//<Master/slave mode //
		| (0 << ETF)           		//<ETF[3:0] bits (External trigger filter) //
		| (0 << ETPS)           		//<ETPS[1:0] bits (External trigger prescaler) //
		| (0 << ECE)           		//<External clock enable //
		| (0 << ETP);           		//<External trigger polarity //

 
 TIM3->DIER = (0 << UIE)           	//<Update interrupt enable //
		| (0 << CC1IE)           		//<Capture/Compare 1 interrupt enable //
		| (0 << CC2IE)           		//<Capture/Compare 2 interrupt enable //
		| (0 << CC3IE)           		//<Capture/Compare 3 interrupt enable //
		| (0 << CC4IE)           		//<Capture/Compare 4 interrupt enable //
		| (0 << COMIE)           		//<COM interrupt enable //
		| (0 << TIE)           		//<Trigger interrupt enable //
		| (0 << BIE)           		//<Break interrupt enable //
		| (0 << UDE)           		//<Update DMA request enable //
		| (0 << CC1DE)           		//<Capture/Compare 1 DMA request enable //
		| (0 << CC2DE)           		//<Capture/Compare 2 DMA request enable //
		| (0 << CC3DE)           		//<Capture/Compare 3 DMA request enable //
		| (0 << CC4DE)           		//<Capture/Compare 4 DMA request enable //
		| (0 << COMDE)           	//<COM DMA request enable //
		| (0 << TDE);           		//<Trigger DMA request enable //

 
 TIM3->SR = 0x0000;
 TIM3->EGR = 0x0000;
 
//----------------------------------------------------------------------------
// TIMx capture/compare usage (x = 2 ~ 5, n = 1 ~ 4)
//
//	CCnS[1:0] :
//		00: CCn channel is configured as output.
//		01: CCn channel is configured as input, ICn is mapped on TI1.
//		10: CCn channel is configured as input, ICn is mapped on TI2.
//		11: CCn channel is configured as input, ICn is mapped on TRC. This mode is working only
//			if an internal trigger input is selected through TS bit (TIMx_SMCR register)
//		Note: 	CCnS bits are writable only when the channel is OFF (CCnE = 0 in TIMx_CCER).
//				Output compare mode
//
//	OCnM[2:0] :
//		000: Frozen
//		001: Set channel n to active level on match. 
//		010: Set channel n to inactive level on match. 
//		011: Toggle - OCnREF toggles when TIMx_CNT=TIMx_CCRn.
//		100: Force inactive level - OCnREF is forced low.
//		101: Force active level - OCnREF is forced high.
//		110: PWM mode 1 - In upcounting, channel n is active as long as TIMx_CNT<TIMx_CCRn
//			else inactive. In downcounting, channel 1 is inactive (OCnREF=0) as long as
//			TIMx_CNT>TIMx_CCRn else active (OCnREF=1).
//		111: PWM mode 2 - In upcounting, channel n is inactive as long as
//			TIMx_CNT<TIMx_CCRn else active. In downcounting, channel n is active as long as
//			TIMx_CNT>TIMx_CCRn else inactive.
//		Note: 1: These bits can not be modified as long as LOCK level 3 has been programmed
//				(LOCK bits in TIMx_BDTR register) and CC1S=00 (the channel is configured in output).
//			  2: In PWM mode 1 or 2, the OCREF level changes only when the result of the
//				comparison changes or when the output compare mode switches from "frozen" mode
//				to "PWM" mode.
//		
//	ICnPSC[1:0] :
//			This bit-field defines the ratio of the prescaler acting on CCn input (ICn).
//			The prescaler is reset as soon as CC1E= 0 (TIMx_CCER register).
//		00: no prescaler, capture is done each time an edge is detected on the capture input.
//		01: capture is done once every 2 events.
//		10: capture is done once every 4 events.
//		11: capture is done once every 8 events.
//		
//	ICnF[3:0] :
//			This bit-field defines the frequency used to sample TIn input and the length of the digital
//			filter applied to TIn. The digital filter is made of an event counter in which N events are
//			needed to validate a transition on the output:
//		0000: No filter, sampling is done at fDTS.
//		0001: fSAMPLING=fCK_INT, N=2.
//		0010: fSAMPLING=fCK_INT, N=4.
//		0011: fSAMPLING=fCK_INT, N=8.
//		0100: fSAMPLING=fDTS/2, N=6.
//		0101: fSAMPLING=fDTS/2, N=8.
//		0110: fSAMPLING=fDTS/4, N=6.
//		0111: fSAMPLING=fDTS/4, N=8.
//		1000: fSAMPLING=fDTS/8, N=6.
//		1001: fSAMPLING=fDTS/8, N=8.
//		1010: fSAMPLING=fDTS/16, N=5.
//		1011: fSAMPLING=fDTS/16, N=6.
//		1100: fSAMPLING=fDTS/16, N=8.
//		1101: fSAMPLING=fDTS/32, N=5.
//		1110: fSAMPLING=fDTS/32, N=6.
//		1111: fSAMPLING=fDTS/32, N=8.
//				Note: 	In current silicon revision, fDTS is replaced in the formula by CK_INT 
//						when ICnF[3:0]= 1, 2 or 3.
//		
// Output compare mode
 TIM3->CCMR1 = (0 << CC1S)          	//!<CC1S[1:0] bits (Capture/Compare 1 Selection) 
		| (0 << OC1FE)           	 	//!<Output Compare 1 Fast enable 
		| (0 << OC1PE)           		//!<Output Compare 1 Preload enable 
		| (0 << OC1M)           		//!<OC1M[2:0] bits (Output Compare 1 Mode) 
		| (0 << OC1CE)           		//!<Output Compare 1Clear Enable 
		| (0 << CC2S)           		//!<CC2S[1:0] bits (Capture/Compare 2 Selection) 
		| (0 << OC2FE)           		//!<Output Compare 2 Fast enable 
		| (0 << OC2PE)           		//!<Output Compare 2 Preload enable 
		| (3 << OC2M)           		//!<OC2M[2:0] bits (Output Compare 2 Mode) 
		| (0 << OC2CE);           	//!<Output Compare 2 Clear Enable 

// Input capture mode
// TIM3->CCMR1 = (0 << CC1S)          	//!<CC1S[1:0] bits (Capture/Compare 1 Selection) 
//		| (0 << IC1PSC)           	//!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) 
//		| (0 << IC1F)           	 	//!<IC1F[3:0] bits (Input Capture 1 Filter) 
//		| (0 << CC2S)           		//!<CC2S[1:0] bits (Capture/Compare 2 Selection) 
//		| (0 << IC2PSC)           	//!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) 
//		| (0 << IC2F);           	 	//!<IC2F[3:0] bits (Input Capture 2 Filter) 

// Output compare mode
 TIM3->CCMR2 = (0 << CC3S)          	//!<CC3S[1:0] bits (Capture/Compare 3 Selection) 
		| (0 << OC3FE)           	 	//!<Output Compare 3 Fast enable 
		| (0 << OC3PE)           		//!<Output Compare 3 Preload enable 
		| (0 << OC3M)           		//!<OC3M[2:0] bits (Output Compare 3 Mode) 
		| (0 << OC3CE)           		//!<Output Compare 3Clear Enable 
		| (0 << CC4S)           		//!<CC4S[1:0] bits (Capture/Compare 4 Selection) 
		| (0 << OC4FE)           		//!<Output Compare 4 Fast enable 
		| (0 << OC4PE)           		//!<Output Compare 4 Preload enable 
		| (0 << OC4M)           		//!<OC4M[2:0] bits (Output Compare 4 Mode) 
		| (0 << OC4CE);           	//!<Output Compare 4 Clear Enable 

// Input capture mode
// TIM3->CCMR2 = (0 << CC3S)          	//!<CC3S[1:0] bits (Capture/Compare 3 Selection) 
//		| (0 << IC3PSC)           	//!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) 
//		| (0 << IC3F)           	 	//!<IC3F[3:0] bits (Input Capture 3 Filter) 
//		| (0 << CC4S)           		//!<CC4S[1:0] bits (Capture/Compare 4 Selection) 
//		| (0 << IC4PSC)           	//!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) 
//		| (0 << IC4F);           	 	//!<IC4F[3:0] bits (Input Capture 4 Filter) 

 TIM3->CCER = (0 << CC1E)           	//<Capture/Compare 1 output enable //
		| (0 << CC1P)           		//<Capture/Compare 1 output Polarity //
		| (0 << CC1NE)           		//<Capture/Compare 1 Complementary output enable //
		| (0 << CC1NP)           		//<Capture/Compare 1 Complementary output Polarity //
		| (1 << CC2E)           		//<Capture/Compare 2 output enable //
		| (0 << CC2P)           		//<Capture/Compare 2 output Polarity //
		| (0 << CC2NE)           		//<Capture/Compare 2 Complementary output enable //
		| (0 << CC2NP)           		//<Capture/Compare 2 Complementary output Polarity //
		| (0 << CC3E)           		//<Capture/Compare 3 output enable //
		| (0 << CC3P)           		//<Capture/Compare 3 output Polarity //
		| (0 << CC3NE)           		//<Capture/Compare 3 Complementary output enable //
		| (0 << CC3NP)           		//<Capture/Compare 3 Complementary output Polarity //
		| (0 << CC4E)           		//<Capture/Compare 4 output enable //
		| (0 << CC4P);           		 //<Capture/Compare 4 output Polarity //

 
 TIM3->CNT = 0x0000;
 
 TIM3->PSC = 3600 - 1;			// 0.5ms clock cycle
 
 TIM3->ARR = 10 - 1;
 
 TIM3->CCR1 = 5;
 TIM3->CCR2 = 5;
 TIM3->CCR3 = 0x0000;
 TIM3->CCR4 = 0x0000;
 TIM3->DCR = 0x0000;
 TIM3->DMAR = 0x0000;

 TIM3->CR1 = (1 << CEN)    	    	//<Counter enable //
		| (0 << UDIS)           		//<Update disable //
		| (0 << URS)          		//<Update request source //
		| (0 << OPM)          		//<One pulse mode //
		| (0 << DIR)          		//<Direction //
		| (0 << CMS)          		//<CMS[1:0] bits (Center-aligned mode selection) //
		| (1 << ARPE)          		//<Auto-reload preload enable //
		| (0 << CKD);          		//<CKD[1:0] bits (clock division) //

}

void	TIM4_Init(void)
{

}


void	SysTick_Init(void)
{
 SysTick->VAL = 0;				// Write this register will clear itself and the settings in 
								//	SysTick->CTRL
								
 SysTick->CTRL = (1 << SysTick_ENABLE)         
				| (1 << SysTick_TICKINT)         	// Counting down to 0 pends the SysTick handler 
				| (1 << SysTick_CLKSOURCE)   	// Clock source. 0 = HCLK/8; 1 = HCLK
				| (0 << SysTick_COUNTFLAG);   	// Count Flag

 SysTick->LOAD = 72000;

// SysTick->CALRB         
// This register is read-only. When clock source is set to HCLK/8 (CLKSOURCE bit is 0) the 
//	TENMS value in this register will be used to generate 1ms tick.
//

}


void	ADC2_Init(void)
{
// NOTE: Remember to program ADC clock in RCC->CFGR

  ADC2->SR = (0 << AWD)                         /*!<Analog watchdog flag */
		| (0 << EOC)                              /*!<End of conversion */
		| (0 << JEOC)                            /*!<Injected channel end of conversion */
		| (0 << JSTRT)                             /*!<Injected channel Start flag */
		| (0 << STRT);                             /*!<Regular channel Start flag */
  	
  ADC2->CR1 = (0 << AWDCH)            /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
			| (0 << EOCIE)           /*!<Interrupt enable for EOC */
			| (0 << AWDIE)              /*!<AAnalog Watchdog interrupt enable */
			| (0 << JEOCIE)          /*!<Interrupt enable for injected channels */
			| (0 << SCAN )           /*!<Scan mode */
			| (0 << AWDSGL)             /*!<Enable the watchdog on a single channel in scan mode */
			| (0 << JAUTO)            /*!<Automatic injected group conversion */
			| (0 << DISCEN)            /*!<Discontinuous mode on regular channels */
			| (0 << JDISCEN)             /*!<Discontinuous mode on injected channels */
			| (0 << DISCNUM )           /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
			| (0 << DUALMOD)             /*!<DUALMOD[3:0] bits (Dual mode selection) */
			| (0 << JAWDEN )            /*!<Analog watchdog enable on injected channels */
			| (0 << AWDEN);		/*!<Analog watchdog enable on regular channels */

  ADC2->CR2 = (0 << ADON)        	//           /*!<A/D Converter ON / OFF */
			| (0 << CONT)        	//          /*!<Continuous Conversion */
			| (0 << CAL)     		//           /*!<A/D Calibration */
			| (0 << RSTCAL)       	//            /*!<Reset Calibration */
			| (0 << DMA)     		//            /*!<Direct Memory access mode */
								//				0: DMA mode disabled
								//				1: DMA mode enabled
			| (0 << ALIGN)   		//            /*!<Data Alignment */
			| (0 << JEXTSEL)		//           /*!<JEXTSEL[2:0] bits (External event select for injected group) */
			| (0 << JEXTTRIG)		//           /*!<External Trigger Conversion mode for injected channels */
			| (0 << EXTSEL)		//            /*!<EXTSEL[2:0] bits (External Event Select for regular group) */
								//			For ADC2 and ADC2, the assigned triggers are:
								//				000: Timer 1 CC1 event
								//				001: Timer 1 CC2 event
								//				010: Timer 1 CC3 event
								//				011: Timer 2 CC2 event
								//				100: Timer 3 TRGO event
								//				101: Timer 4 CC4 event
								//				110: EXTI line11/TIM8_TRGO event (TIM8_TRGO is available only in high-density devices)
								//				111: SWSTART
			| (0 << EXTTRIG)		//              /*!<External Trigger Conversion mode for regular channels */
			| (0 << JSWSTART)	//            /*!<Start Conversion of injected channels */
			| (0 << SWSTART)		//              /*!<Start Conversion of regular channels */
			| (0 << TSVREFE);		//              /*!<Temperature Sensor and VREFINT Enable */

 // Sample time selection
 // SMPx[2:0]:
 //		000: 1.5 cycles
 //		001: 7.5 cycles
 //		010: 13.5 cycles
 //		011: 28.5 cycles
 //		100: 41.5 cycles
 //		101: 55.5 cycles
 //		110: 71.5 cycles
 //		111: 239.5 cycles
 ADC2->SMPR1 = (0 << SMP10) 	//           /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
			| (0 << SMP11) 	//            /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
			| (0 << SMP12) 	//              /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
			| (0 << SMP13)	//                 /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
			| (0 << SMP14)	//               /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
			| (0 << SMP15)	//            /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
			| (0 << SMP16) 	//             /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
			| (0 << SMP17); 	//               /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */

 ADC2->SMPR2 = (0 << SMP0 )  	//        /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
			| (0 << SMP1)   	//            /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
			| (0 << SMP2)   	//              /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
			| (0 << SMP3) 	//             /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
			| (0 << SMP4 )  	//              /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
			| (0 << SMP5)   	//            /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
			| (0 << SMP6)   	//           /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
			| (0 << SMP7)  	//           /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
			| (0 << SMP8)  	//          /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
			| (0 << SMP9);  	//            /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
  
  ADC2->JOFR1 = 0x0000;
  ADC2->JOFR2 = 0x0000;
  ADC2->JOFR3 = 0x0000;
  ADC2->JOFR4 = 0x0000;
  
  ADC2->HTR = 0x0FFF;
  ADC2->LTR = 0x0000;

 //	L[3:0]: Regular channel sequence length, i.e. number of channels in the sequence.
 //		These bits are written by software to define the total number of conversions in the regular
 //		channel conversion sequence.
 //			0000: 1 conversion
 //			0001: 2 conversions
 //			.....
 //			1111: 16 conversions 
 //	SQn[4:0]: The order of conversion in regular sequence
 //		These bits are written by software with the channel number (0..17) assigned as the n-th conversion in the
 //		sequence to be converted.
 //
 ADC2->SQR1 = (0 << SQ13 )   //            /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
			| (0 << SQ14)     	//              /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
			| (0 << SQ15)    	//                /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
			| (0 << SQ16)    	//              /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
			| (0 << L );     	//             /*!<L[3:0] bits (Regular channel sequence length) */
  
  ADC2->SQR2 = (0 << SQ7)      //               /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
			| (0 << SQ8)       	//              /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
			| (0 << SQ9)     	//                /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
			| (0 << SQ10)     	//               /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
			| (0 << SQ11)      	//               /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
			| (0 << SQ12);     	//               /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
  
  ADC2->SQR3 = (0 << SQ1)     	//             /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
			| (0 << SQ2)        	//            /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
			| (0 << SQ3)      	//              /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
			| (0 << SQ4)      	//             /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
			| (0 << SQ5)      	//              /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
			| (0 << SQ6);       	//             /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
			
 //	JL[1:0]: Injected sequence length
 //		These bits are written by software to define the total number of conversions in the injected
 //		channel conversion sequence.
 //			00: 1 conversion
 //			01: 2 conversions
 //			10: 3 conversions
 //			11: 4 conversions
 //	JSQ4[4:0]: 4th conversion in injected sequence
 //		These bits are written by software with the channel number (0..17) assigned as the 4th in
 //		the sequence to be converted.
 //		Note: Unlike a regular conversion sequence, if JL[1:0] length is less than four, the channels
 //				are converted in a sequence starting from (4-JL). Example: ADC_JSQR[21:0] = 10
 //				00011 00011 00111 00010 means that a scan conversion will convert the following
 //				channel sequence: 7, 3, 3. (not 2, 7, 3) 
 //
  ADC2->JSQR = (0 << JSQ1)     //            /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */  
			| (0 << JSQ2)       	//             /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
			| (0 << JSQ3)       	//             /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
			| (0 << JSQ4)       	//              /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
			| (0 << JL);        	//            /*!<JL[1:0] bits (Injected Sequence length) */

  // These registers are read-only
//  ADC2->JDR1;
//  ADC2->JDR2;
//  ADC2->JDR3;
//  ADC2->JDR4;
//  ADC2->DR;

 // Do calibration
 ADC2->CR2 |= (1 << CAL);     		
 while(!BitTest(ADC2->CR2, (1 << CAL))) {
 	// Wait for end of  calibration
 	}
 
 // Start ADC (the first ADON set turn on ADC power)
 ADC2->CR2 |= (1 << ADON);        	//           /*!<A/D Converter ON / OFF */
}


U16	ADC_Poll(ADC_TypeDef * adc, U8 chn)
{
 // Assuming that the ADC refered has been properly initialized with channel and sample time selected.
  adc->SQR3 = (chn << SQ1);     	//             /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
 
 // Start conversion
 adc->CR2 |= (1 << ADON); 
 while(!BitTest(adc->SR, (1 << EOC))) {
 	// Wait for end of conversion
 	}
 return (adc->DR);
}


void	TFT_Init_Ili9341(void)
{
 U8  tmp;

 // Reset TFT controller (Ili9341)
 SetToHigh(TFT_nRESET_Port, (1 << TFT_nRESET_Bit));
 Delay(5000);	// About 1.1ms
 SetToLow(TFT_nRESET_Port, (1 << TFT_nRESET_Bit));
 Delay(65000);	// About 15ms
 SetToHigh(TFT_nRESET_Port, (1 << TFT_nRESET_Bit));
 tmp = 10;
 while(tmp) {
 	Delay(65535);
	tmp--;
 	}
 
	write_comm(0xcf); 
	write_data(0x00);
	write_data(0xC1);
	write_data(0x30);

	write_comm(0xed); 
	write_data(0x67);
	write_data(0x03);
	write_data(0x12);
	write_data(0x81);

	write_comm(0xcb); 
	write_data(0x39);
	write_data(0x2c);
	write_data(0x00);
	write_data(0x34);
	write_data(0x02);

	write_comm(0xea); 
	write_data(0x00);
	write_data(0x00);

	write_comm(0xe8); 
	write_data(0x85);
	write_data(0x0a);
	write_data(0x78);

	write_comm(0xF7); 
	write_data(0x20);

	write_comm(0xC0); //Power control
	write_data(0x26); //VRH[5:0]

	write_comm(0xC1); //Power control
	write_data(0x01); //SAP[2:0];BT[3:0]

	write_comm(0xC5); //VCM control
	write_data(0x2b);
	write_data(0x2F);

	write_comm(0xc7); 
	write_data(0xc7);

	write_comm(0x3A); 
	write_data(0x55);

	write_comm(0x36); // Memory Access Control
//	write_data(0x08);
	write_data(0x20);
	
	write_comm(0xB1); // Frame Rate Control
	write_data(0x00);
	write_data(0x18);
	
	write_comm(0xB6); // Display Function Control
	write_data(0x0a);
//	write_data(0x82);	// Normal orientation
	write_data(0xE2);		// Rotate 180 degree
	
	write_comm(0xF2); // 3Gamma Function Disable
	write_data(0x00);
	write_comm(0x26); //Gamma curve selected
	write_data(0x01);
	write_comm(0xE0); //Set Gamma
	write_data(0x0f);
	write_data(0x1d);
	write_data(0x1a);
	write_data(0x09);
	write_data(0x0f);
	write_data(0x09);
	write_data(0x46);
	write_data(0x88);
	write_data(0x39);
	write_data(0x05);
	write_data(0x0f);
	write_data(0x03);
	write_data(0x07);
	write_data(0x05);
	write_data(0x00);

	write_comm(0XE1); //Set Gamma
	write_data(0x00);
	write_data(0x22);
	write_data(0x25);
	write_data(0x06);
	write_data(0x10);
	write_data(0x06);
	write_data(0x39);
	write_data(0x22);
	write_data(0x4a);
	write_data(0x0a);
	write_data(0x10);
	write_data(0x0c);
	write_data(0x38);
	write_data(0x3a);
	write_data(0x0F);

	write_comm(0x11); //Exit Sleep
//	delay(120);
	 tmp = 100;
	 while(tmp) {
	 	Delay(50000);
		tmp--;
	 	}
	write_comm(0x29); //display on	
//	write_comm(0x2C);	

 Delay(50000);
 Delay(50000);
 
}


void	write_comm(U8 commport)
{
 // Set TFT_nCS low
 SetToLow(TFT_nCS_Port, (1 << TFT_nCS_Bit));
 // Set up to access Index Register (RS == 0)
 SetToLow(TFT_RS_Port, (1 << TFT_RS_Bit));
// Delay(2);

 TFT_Port = (TFT_Port & 0xFF00) | commport;
 SetToLow(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 SetToHigh(TFT_nWR_Port, (1 << TFT_nWR_Bit));

 // Set up to access Data Register (RS == 1)
 SetToHigh(TFT_RS_Port, (1 << TFT_RS_Bit));
// Delay(2);

 // Set TFT_nCS high
 SetToHigh(TFT_nCS_Port, (1 << TFT_nCS_Bit));
 
}

void write_data(U8 data)
{
 // Set TFT_nCS low
 SetToLow(TFT_nCS_Port, (1 << TFT_nCS_Bit));

 // Set up to access Data Register (RS == 1)
 SetToHigh(TFT_RS_Port, (1 << TFT_RS_Bit));

 TFT_Port = (TFT_Port & 0xFF00) | data;
 SetToLow(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 SetToHigh(TFT_nWR_Port, (1 << TFT_nWR_Bit));

 // Set TFT_nCS high
 SetToHigh(TFT_nCS_Port, (1 << TFT_nCS_Bit));
 
}


U32	TFT_ReadID_Ili9341(void)
{
 U32 tmp;
 U8 tmp0, tmp1;

 write_comm(0xD3);

 // Set TFT_nCS low
 SetToLow(TFT_nCS_Port, (1 << TFT_nCS_Bit));
 // Set up to access Data Register (RS == 1)
 SetToHigh(TFT_RS_Port, (1 << TFT_RS_Bit));

 // Set PB[0:7] input mode
 GPIOB->CRL = ((GPIO_CNF_Floating | GPIO_Mode_In) << (0*4))		// TFT port - D0
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (1*4))		// TFT port - D1
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (2*4))		// TFT port - D2
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (3*4))		// TFT port - D3
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (4*4))		// TFT port - D4
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (5*4))		// TFT port - D5
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (6*4))		// TFT port - D6	
 			|((GPIO_CNF_Floating | GPIO_Mode_In) << (7*4));		// TFT port - D7		
 Delay(100);

 // Read ID into tmp
 tmp0 = 0;
 tmp = 0;
 while(tmp0 < 4) {
 	SetToLow(TFT_nRD_Port, (1 << TFT_nRD_Bit));
 	Delay(10);
 	tmp1 = TFT_Port_In;
 	SetToHigh(TFT_nRD_Port, (1 << TFT_nRD_Bit));
 	tmp <<= 8;
 	tmp |= tmp1;	// The first one is dummy read
	tmp0++;
 	}

 // Restore PB[0:7] to output mode
 GPIOB->CRL = ((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (0*4))		// TFT port - D0
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (1*4))		// TFT port - D1
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (2*4))		// TFT port - D2
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (3*4))		// TFT port - D3
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (4*4))		// TFT port - D4
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (5*4))		// TFT port - D5
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (6*4))		// TFT port - D6	
 			|((GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (7*4));		// TFT port - D7		
 
 // Set TFT_nCS high
 SetToHigh(TFT_nCS_Port, (1 << TFT_nCS_Bit));
 
 return tmp;
 
}


void	TFT_Init_Ili9325(void)
{
 U16 tmp;
 
 // Reset TFT controller (Ili9341)
 SetToHigh(TFT_nRESET_Port, (1 << TFT_nRESET_Bit));
 Delay(5000);	// About 1.1ms
 SetToLow(TFT_nRESET_Port, (1 << TFT_nRESET_Bit));
 Delay(65000);	// About 15ms
 tmp = 10;
 while(tmp) {
 	Delay(65535);
	tmp--;
 	}
 SetToHigh(TFT_nRESET_Port, (1 << TFT_nRESET_Bit));

 tmp = 10;
 while(tmp) {
 	Delay(65535);
	tmp--;
 	}
 
 // Set nCS LOW
// TFT_nCS_Port->BRR |= (1 << TFT_nCS_Bit);
 Delay(5000);

// Normal orientation of screen
//                 
//    -----------------
//   |            YU            |
//   |                            |
//   |                            |
//   |                            |
//   |                            |
//   |                            |
//   | XL                   XR|
//   |                            |
//   |                            |
//   |                            |
//   |                            |
//   |                            |
//   |             YD           |
//    -----------------
//       ||||||||||||||
//       ||||||||||||||
//
// Orientation used in this application
//
//        ---------------------------
//   --|                    XL                      |
//   --|                                              |
//   --|                                              |
//   --|                                              |
//   --| YD                                    YU |
//   --|                                              |
//   --|                                              |
//   --|                    XR                      |
//       ---------------------------
/*
	LCD_WriteReg(0x00E5,0x78F0); 
	LCD_WriteReg(0x0001,0x0100); 
	LCD_WriteReg(0x0002,0x0700); 
	LCD_WriteReg(0x0003,0x1030); 
	LCD_WriteReg(0x0004,0x0000); 
	LCD_WriteReg(0x0008,0x0202);  
	LCD_WriteReg(0x0009,0x0000);
	LCD_WriteReg(0x000A,0x0000); 
	LCD_WriteReg(0x000C,0x0000); 
	LCD_WriteReg(0x000D,0x0000);
	LCD_WriteReg(0x000F,0x0000);
*/
	
 // Start initialization sequence
 TFT_CmdWrite(0x00E5, 0x78F0);
 TFT_CmdWrite(0x0001, 0x0100);	//	LCD_Write_Com(0x00,0x01); LCD_Write_Data(0x00,0x00); // set SS=0 and SM bit 0x0100-------------
 TFT_CmdWrite(0x0002, 0x0700);	//	LCD_Write_Com(0x00,0x02); LCD_Write_Data(0x07,0x00); // set 1 line inversion 
// TFT_CmdWrite(0x0003, 0x1018);	//	LCD_Write_Com(0x00,0x03); LCD_Write_Data(0x10,0x28); // 横屏,方向控制 ID=10,AM=1---------------
 TFT_CmdWrite(0x0003, 0x0038);	//	LCD_Write_Com(0x00,0x03); LCD_Write_Data(0x10,0x28); // 横屏,方向控制 ID=10,AM=1---------------
// TFT_CmdWrite(0x0003, 0x1030);	//	//LCD_Write_Com(0x00,0x03); LCD_Write_Data(0x10,0x30); // 竖屏,set GRAM write direction and BGR=1. 
 TFT_CmdWrite(0x0004, 0x0000);	//	LCD_Write_Com(0x00,0x04); LCD_Write_Data(0x00,0x00); // Resize register 
 TFT_CmdWrite(0x0008, 0x0207);	//	LCD_Write_Com(0x00,0x08); LCD_Write_Data(0x02,0x07); // set the back porch and front porch 
 TFT_CmdWrite(0x0009, 0x0000);	//	LCD_Write_Com(0x00,0x09); LCD_Write_Data(0x00,0x00); // set non-display area refresh cycle ISC[3:0] 
 TFT_CmdWrite(0x000A, 0x0000);	//	LCD_Write_Com(0x00,0x0A); LCD_Write_Data(0x00,0x00); // FMARK function 
 TFT_CmdWrite(0x000C, 0x0000);	//	LCD_Write_Com(0x00,0x0C); LCD_Write_Data(0x00,0x00); // RGB interface setting 
 TFT_CmdWrite(0x000D, 0x0000);	//	LCD_Write_Com(0x00,0x0D); LCD_Write_Data(0x00,0x00); // Frame marker Position 
 TFT_CmdWrite(0x000F, 0x0000);	//	LCD_Write_Com(0x00,0x0F); LCD_Write_Data(0x00,0x00); // RGB interface polarity 

 // Power up sequence	
 TFT_CmdWrite(0x0010, 0x0000);	//	LCD_Write_Com(0x00,0x10); LCD_Write_Data(0x00,0x00); // SAP, BT[3:0], AP, DSTB, SLP, STB 
 TFT_CmdWrite(0x0011, 0x0007);	//	LCD_Write_Com(0x00,0x11); LCD_Write_Data(0x00,0x07); // DC1[2:0], DC0[2:0], VC[2:0] 
 TFT_CmdWrite(0x0012, 0x0000);	//	LCD_Write_Com(0x00,0x12); LCD_Write_Data(0x00,0x00); // VREG1OUT voltage 
 TFT_CmdWrite(0x0013, 0x0000);	//	LCD_Write_Com(0x00,0x13); LCD_Write_Data(0x00,0x00); // VDV[4:0] for VCOM amplitude 
 TFT_CmdWrite(0x0007, 0x0001);	//	LCD_Write_Com(0x00,0x07); LCD_Write_Data(0x00,0x01); 
 TFT_CmdWrite(0x0007, 0x0020);	//	LCD_Write_Com(0x00,0x07); LCD_Write_Data(0x00,0x20); 
 Delay(50000);					//	delayms(50); // Dis-charge capacitor power voltage 
 Delay(50000);
 Delay(50000);
 TFT_CmdWrite(0x0010, 0x1290);	//	LCD_Write_Com(0x00,0x10); LCD_Write_Data(0x12,0x90); // 1490//SAP, BT[3:0], AP, DSTB, SLP, STB 
 TFT_CmdWrite(0x0011, 0x0221);	//	LCD_Write_Com(0x00,0x11); LCD_Write_Data(0x02,0x21); // DC1[2:0], DC0[2:0], VC[2:0] 
 Delay(50000);					//	delayms(50);// delayms 50ms 
 Delay(50000);
 Delay(50000);
 TFT_CmdWrite(0x0012, 0x0081);	//	LCD_Write_Com(0x00,0x12); LCD_Write_Data(0x00,0x81); //001C// Internal reference voltage= Vci; 
 Delay(50000);					//	delayms(50); // delayms 50ms 
 Delay(50000);
 Delay(50000);
 TFT_CmdWrite(0x0013, 0x1500);	//	LCD_Write_Com(0x00,0x13); LCD_Write_Data(0x15,0x00); //0x1000//1400   Set VDV[4:0] for VCOM amplitude  1A00 
 TFT_CmdWrite(0x0029, 0x000C);	//	LCD_Write_Com(0x00,0x29); LCD_Write_Data(0x00,0x0C); //0x0012 //001a  Set VCM[5:0] for VCOMH  //0x0025  0034 
 TFT_CmdWrite(0x002B, 0x000D);	//	LCD_Write_Com(0x00,0x2B); LCD_Write_Data(0x00,0x0D); // Set Frame Rate   000C 
 Delay(50000);					//	delayms(50);// delayms 50ms
 Delay(50000);
 Delay(50000);
	
 TFT_CmdWrite(0x0020, 0x0000);	//	LCD_Write_Com(0x00,0x20); LCD_Write_Data(0x00,0x00); // GRAM horizontal Address 	*
 TFT_CmdWrite(0x0021, 0x0100);	//	LCD_Write_Com(0x00,0x21); LCD_Write_Data(0x01,0x00); // GRAM Vertical Address 		*
	
 // Adjust the Gamma Curve 
 TFT_CmdWrite(0x0030, 0x0303);	//	LCD_Write_Com(0x00,0x30); LCD_Write_Data(0x03,0x03); 
 TFT_CmdWrite(0x0031, 0x0006);	//	LCD_Write_Com(0x00,0x31); LCD_Write_Data(0x00,0x06); 
 TFT_CmdWrite(0x0032, 0x0001);	//	LCD_Write_Com(0x00,0x32); LCD_Write_Data(0x00,0x01); 
 TFT_CmdWrite(0x0035, 0x0204);	//	LCD_Write_Com(0x00,0x35); LCD_Write_Data(0x02,0x04); 
 TFT_CmdWrite(0x0036, 0x0004);	//	LCD_Write_Com(0x00,0x36); LCD_Write_Data(0x00,0x04);//0207 
 TFT_CmdWrite(0x0037, 0x0407);	//	LCD_Write_Com(0x00,0x37); LCD_Write_Data(0x04,0x07);//0306 
 TFT_CmdWrite(0x0038, 0x0000);	//	LCD_Write_Com(0x00,0x38); LCD_Write_Data(0x00,0x00);//0102 
 TFT_CmdWrite(0x0039, 0x0404);	//	LCD_Write_Com(0x00,0x39); LCD_Write_Data(0x04,0x04);//0707 
 TFT_CmdWrite(0x003C, 0x0402);	//	LCD_Write_Com(0x00,0x3C); LCD_Write_Data(0x04,0x02);//0702 
 TFT_CmdWrite(0x003D, 0x0004);	//	LCD_Write_Com(0x00,0x3D); LCD_Write_Data(0x00,0x04);//1604 

 // Set GRAM area
 TFT_CmdWrite(0x0050, 0x0000);	//	LCD_Write_Com(0x00,0x50); LCD_Write_Data(0x00,0x00); //00 Horizontal GRAM Start Address*
 TFT_CmdWrite(0x0051, 0x00EF);	//	LCD_Write_Com(0x00,0x51); LCD_Write_Data(0x00,0xEF); //ef Horizontal GRAM End Address 	*
 TFT_CmdWrite(0x0052, 0x0000);	//	LCD_Write_Com(0x00,0x52); LCD_Write_Data(0x00,0x00); // Vertical GRAM Start Address	*
 TFT_CmdWrite(0x0053, 0x013F);	//	LCD_Write_Com(0x00,0x53); LCD_Write_Data(0x01,0x3F); // Vertical GRAM Start Address 	*
 TFT_CmdWrite(0x0060, 0x2700);	//	LCD_Write_Com(0x00,0x60); LCD_Write_Data(0x27,0x00); // Gate Scan Line GS=0;0xa700------------------
 TFT_CmdWrite(0x0061, 0x0001);	//	LCD_Write_Com(0x00,0x61); LCD_Write_Data(0x00,0x01); // NDL,VLE, REV 
 TFT_CmdWrite(0x006A, 0x0000);	//	LCD_Write_Com(0x00,0x6A); LCD_Write_Data(0x00,0x00); // set scrolling line 

 //-------------- Partial Display Control ---------
 TFT_CmdWrite(0x0080, 0x0000);	//	LCD_Write_Com(0x00,0x80); LCD_Write_Data(0x00,0x00); 
 TFT_CmdWrite(0x0081, 0x0000);	//	LCD_Write_Com(0x00,0x81); LCD_Write_Data(0x00,0x00); 
 TFT_CmdWrite(0x0082, 0x0000);	//	LCD_Write_Com(0x00,0x82); LCD_Write_Data(0x00,0x00); 
 TFT_CmdWrite(0x0083, 0x0000);	//	LCD_Write_Com(0x00,0x83); LCD_Write_Data(0x00,0x00); 
 TFT_CmdWrite(0x0084, 0x0000);	//	LCD_Write_Com(0x00,0x84); LCD_Write_Data(0x00,0x00); 
 TFT_CmdWrite(0x0085, 0x0000);	//	LCD_Write_Com(0x00,0x85); LCD_Write_Data(0x00,0x00); 

 //-------------- Panel Control ------------------- 
 TFT_CmdWrite(0x0090, 0x0010);	//	LCD_Write_Com(0x00,0x90); LCD_Write_Data(0x00,0x10); 
 TFT_CmdWrite(0x0092, 0x0600);	//	LCD_Write_Com(0x00,0x92); LCD_Write_Data(0x06,0x00); 
 TFT_CmdWrite(0x0007, 0x0133);	//	LCD_Write_Com(0x00,0x07); LCD_Write_Data(0x01,0x33); // 262K color and display ON 
	
//	LCD_CS_H;

 Delay(50000);
 Delay(50000);
 Delay(50000);

}

// Write Data to Reg in Ili9325 
void	TFT_CmdWrite(U16 Reg, U16 Data)
{
 // Set TFT_nCS low
 SetToLow(TFT_nCS_Port, (1 << TFT_nCS_Bit));
 // Set up to access Index Register (RS == 0)
 SetToLow(TFT_RS_Port, (1 << TFT_RS_Bit));
 Delay(2);

 TFT_Port = (TFT_Port & 0xFF00) | (Reg >> 8);
 SetToLow(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 SetToHigh(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 TFT_Port = (TFT_Port & 0xFF00) | Reg;
 SetToLow(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 SetToHigh(TFT_nWR_Port, (1 << TFT_nWR_Bit));

 // Set up to access Data Register (RS == 1)
 SetToHigh(TFT_RS_Port, (1 << TFT_RS_Bit));
 Delay(2);
 
 TFT_Port = (TFT_Port & 0xFF00) | (Data >> 8);
 SetToLow(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 SetToHigh(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 TFT_Port = (TFT_Port & 0xFF00) | Data;
 SetToLow(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 SetToHigh(TFT_nWR_Port, (1 << TFT_nWR_Bit));

 // Set TFT_nCS high
 SetToHigh(TFT_nCS_Port, (1 << TFT_nCS_Bit));

}

void	TFT_AccessGRAM(void)
{

 // Set TFT_nCS low
 SetToLow(TFT_nCS_Port, (1 << TFT_nCS_Bit));
 // Set up to access Index Register (RS == 0)
 SetToLow(TFT_RS_Port, (1 << TFT_RS_Bit));
 Delay(2);

 TFT_Port = (TFT_Port & 0xFF00) | 0x00;
 SetToLow(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 SetToHigh(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 Delay(2);
 TFT_Port = (TFT_Port & 0xFF00) | 0x22;
 SetToLow(TFT_nWR_Port, (1 << TFT_nWR_Bit));
 SetToHigh(TFT_nWR_Port, (1 << TFT_nWR_Bit));

}

void	TFT_AccessGRAM_End(void)
{
 // Set TFT_nCS high
 SetToHigh(TFT_nCS_Port, (1 << TFT_nCS_Bit));
}

void assert_failed(U8 * file, U32 line)
//void assert_failed((U8 *) file, U32 line)
{
}


/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

// NVIC_SetVectorTable(NVIC_VectTab_RAM, 0);
 NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);

  // Enable the TIM1 Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 


  // Enable the ADC1 Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
 
  // Enable the DMA1 channel1 Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  // Enable the USART1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


}


void	OutputVSen(void)
{
 U16 tmp;

 tmp = (VSenCtrlTab[GetVSen() - VSenMin]) & VSen_Bits;
 VSen_Port = (VSen_Port & ~VSen_Bits) | tmp;
 
}

void	I2C_Start(void)
{
 SetSDA_Out();

 SetSDA_H;
 SetSCL_L;
 Delay(I2C_QuadCycle);

 SetSCL_H;
 Delay(I2C_QuadCycle);
 SetSDA_L;
 Delay(I2C_QuadCycle);

 SetSCL_L;
 Delay(I2C_QuadCycle);

 SetSDA_In();
 
}

void	I2C_Stop(void)
{
 SetSDA_Out();

 SetSDA_L;
 SetSCL_L;
 Delay(I2C_QuadCycle);

 SetSCL_H;
 Delay(I2C_QuadCycle);
 SetSDA_H;
 Delay(I2C_QuadCycle);

 SetSCL_L;
 Delay(I2C_QuadCycle);

 SetSDA_In();
 
}

void	I2C_SendByte(U8 byte)
{
 U8	tmp0;

 SetSDA_Out();
 
 tmp0 = 0;
 while(tmp0 < 8) {
	if(BitTest(byte, 0x80)) {
		SetSDA_H;
		}
	else {
		SetSDA_L;
		}
	Delay(I2C_QuadCycle);
	SetSCL_H;
	Delay(I2C_QuadCycle);
	Delay(I2C_QuadCycle);
	SetSCL_L;
	if(tmp0 == 7) {
		// Last bit
		SetSDA_In();
		}
	Delay(I2C_QuadCycle);
	byte <<= 1;
	tmp0++;
 	}
}

U8	I2C_RecvByte(void)
{
 U8	tmp0;
 U8	byte;
 
 tmp0 = 0;
 while(tmp0 < 8) {
	Delay(I2C_QuadCycle);
	SetSCL_H;
	Delay(I2C_QuadCycle);
	byte <<= 1;
	if(GetSDA) {
		byte |= 0x01;
		}
	Delay(I2C_QuadCycle);
	SetSCL_L;
	Delay(I2C_QuadCycle);
	tmp0++;
 	}
 return byte;
}

// Return non-zero if valid ACK detected
U8	I2C_CheckAck(void)
{
 U8 tmp;
 
 Delay(I2C_QuadCycle);
 SetSCL_H;
 Delay(I2C_QuadCycle);
 tmp = 0x01;
 if(GetSDA) {
 	tmp = 0x00;
 	}
 Delay(I2C_QuadCycle);
 SetSCL_L;
 Delay(I2C_QuadCycle);
 return tmp;
 
}

void	I2C_Ack(void)
{
 SetSDA_Out();

 SetSDA_L;
 Delay(I2C_QuadCycle);
 SetSCL_H;
 Delay(I2C_QuadCycle);
 Delay(I2C_QuadCycle);
 SetSCL_L;
 Delay(I2C_QuadCycle);

 SetSDA_In();
}

void	I2C_Nak(void)
{
 SetSDA_Out();

 SetSDA_H;
 Delay(I2C_QuadCycle);
 SetSCL_H;
 Delay(I2C_QuadCycle);
 Delay(I2C_QuadCycle);
 SetSCL_L;
 Delay(I2C_QuadCycle);

 SetSDA_In();
}

void	I2C_ReSync(void)
{
 U8 tmp0;
 
 I2C_Start();

 // 9 cycles of SCL with SDA high
 SetSCL_H;
 
 tmp0 = 0;
 while(tmp0 < 9) {
 	Delay(I2C_QuadCycle);
 	SetSCL_H;
 	Delay(I2C_QuadCycle);
 	Delay(I2C_QuadCycle);
 	SetSCL_L;
 	Delay(I2C_QuadCycle);

	tmp0++;
 	}

 I2C_Start();
 I2C_Stop();
 
}

void	SetSDA_In(void)
{
 I2C_SDA_Port->CRH &= ~(0x000F << (I2C_SDA_Bit - 8) * 4);
 I2C_SDA_Port->CRH |= ((GPIO_CNF_Floating | GPIO_Mode_In) << (I2C_SDA_Bit - 8) * 4);
}

void SetSDA_Out(void)
{
 I2C_SDA_Port->CRH &= ~(0x000F << (I2C_SDA_Bit - 8) * 4);
 I2C_SDA_Port->CRH |= (GPIO_CNF_GP_PP | GPIO_Mode_Out50M) << (I2C_SDA_Bit - 8) * 4;
}

