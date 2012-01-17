/******************************************************************************
 *  FILE		 	: uart_dma.h
 *  DESCRIPTION  	: uart(RS-232) configuration using dma
 *  CPU TYPE     	: dsPIC33FJ256MC710
 *  AUTHOR	     	: Antonio Camacho Santiago
 *  PROJECT	     	: DPI2007-61527
 *  COMPANY	     	: Automatic Control Department,
 *  				  Technical University of Catalonia
 *
 *  REVISION HISTORY:
 *			 VERSION: 0.1
 *     		  AUTHOR: Antonio Camacho Santiago
 * 				DATE: 27th April 2010
 * 			COMMENTS: Adapted from Microchip CE214_UART_loopback demo
 *****************************************************************************/

#ifndef __INCLUDE_UART_DMA_H__
#define __INCLUDE_UART_DMA_H__


//  Allocate two buffers for DMA transfers
#define OUTPUT_DATA_SIZE 71//71//23 //Output buffer size
#define INPUT_DATA_SIZE 8  //Input buffer size
extern unsigned char OutBuffer[OUTPUT_DATA_SIZE] __attribute__((space(dma)));
extern unsigned char InBufferA[INPUT_DATA_SIZE] __attribute__((space(dma)));


////These macros don't work properly, implemented by hand above:
//#define FCY 40000000   //40MIPS Fcy=Foscillator/2
//#define BRGVAL   ((FCY/BITRATE1)/16)-1 //With BRGH=0-->Slow Bitrate
//#define BRGVAL   ((FCY/BITRATE1)/4)-1  //With BRGH=1-->Fast Bitrate
//
////Tested values for BRGVAL:
//#define BRGVAL 21 //460800bps with BRGH=1
//#define BRGVAL 43 //230400bps with BRGH=1
//#define BRGVAL 86 //115200bps with BRGH=1

#define BRGVAL 21

extern float k_updated[2];
extern void cfgUart1(void);
extern void cfgDma4UartTx(void);
extern void cfgDma5UartRx(void);
extern void UART1_DMA_config(void);

#endif




// UART
//#if defined(STACK_USE_UART)
//	UARTTX_TRIS = 0;
//	UARTRX_TRIS = 1;
//	UMODE = 0x8000;			// Set UARTEN.  Note: this must be done before setting UTXEN
//
//	#if defined(__C30__)
//		USTA = 0x0400;		// UTXEN set
//		#define CLOSEST_UBRG_VALUE ((GetPeripheralClock()+8ul*BAUD_RATE)/16/BAUD_RATE-1)
//		#define BAUD_ACTUAL (GetPeripheralClock()/16/(CLOSEST_UBRG_VALUE+1))
//	#else	//defined(__C32__)
//		USTA = 0x00001400;		// RXEN set, TXEN set
//		#define CLOSEST_UBRG_VALUE ((GetPeripheralClock()+8ul*BAUD_RATE)/16/BAUD_RATE-1)
//		#define BAUD_ACTUAL (GetPeripheralClock()/16/(CLOSEST_UBRG_VALUE+1))
//	#endif
//
//	#define BAUD_ERROR ((BAUD_ACTUAL > BAUD_RATE) ? BAUD_ACTUAL-BAUD_RATE : BAUD_RATE-BAUD_ACTUAL)
//	#define BAUD_ERROR_PRECENT	((BAUD_ERROR*100+BAUD_RATE/2)/BAUD_RATE)
//	#if (BAUD_ERROR_PRECENT > 3)
//		#warning UART frequency error is worse than 3%
//	#elif (BAUD_ERROR_PRECENT > 2)
//		#warning UART frequency error is worse than 2%
//	#endif
//
//	UBRG = CLOSEST_UBRG_VALUE;
//#endif
