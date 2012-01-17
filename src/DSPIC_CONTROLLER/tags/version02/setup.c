/******************************************************************************
 *  FILE		 	: setup.h
 *  DESCRIPTION  	: FLEX FULL board configuration
 *  CPU TYPE     	: dsPIC33FJ256MC710
 *  AUTHOR	     	: Antonio Camacho Santiago
 *  PROJECT	     	: DPI2007-61527
 *  COMPANY	     	: Automatic Control Department,
 *  				  Technical University of Catalonia
 *
 *  REVISION HISTORY:
 *			 VERSION: 0.1
 *     		  AUTHOR: Antonio Camacho Santiago
 * 				DATE: April 2010
 * 			COMMENTS:
 *****************************************************************************/

#include "ee.h"
#include "cpu/pic30/inc/ee_irqstub.h"
#include "setup.h"
//#include "uart_dma.h"
#include "e_can1.h"

/* Program the Timer1 peripheral to raise interrupts */
void T1_program(void)
{
	T1CON = 0;			/* Stops the Timer1 and reset control register	*/
	TMR1  = 0;			/* Clear contents of the timer register	*/
	PR1   = 0x9c40;		/* Load the Period register with the value of 1ms	*/
	IPC0bits.T1IP = 5;	/* Set Timer1 priority to 1. Higher values set higher priorities */
	IFS0bits.T1IF = 0;	/* Clear the Timer1 interrupt status flag	*/
	IEC0bits.T1IE = 1;	/* Enable Timer1 interrupts		*/
	T1CONbits.TON = 1;	/* Start Timer1 with prescaler settings at 1:1
						* and clock source set to the internal instruction cycle */
}

/* Clear the Timer1 interrupt status flag */
void T1_clear(void)
{
	IFS0bits.T1IF = 0;
}

/* This is an ISR Type 2 which is attached to the Timer 1 peripheral IRQ pin
 * The ISR simply calls CounterTick to implement the timing reference */
static unsigned int my_time=0;
ISR2(_T1Interrupt)
{
	/* clear the interrupt source */
	T1_clear();

	my_time++;/*Instead of using the TMR8 & TMR9 we use this instruction to get time*/

	/* count the interrupts, waking up expired alarms */
	CounterTick(myCounter);
}

/******************************************************************************
 * Function:	Led_config()
 * Description:	Configures  FLEX FULL orange led (Jumper 4 must be closed)
******************************************************************************/
void Led_config(void)
{
	/* set LED (LEDSYS/RB14) drive state low */
	LATBbits.LATB14 = 0;
	/* set LED pin (LEDSYS/RB14) as output */
	TRISBbits.TRISB14 = 0;
}

/******************************************************************************
 * Function:	Signals_config()
 * Description:	Configures pin (AN10/RB10)-->(CON6/Pin28) from the FLEX FULL
 * 				to get execution times with oscilloscope
******************************************************************************/
void Signals_config(void)
{
	/* set pin (AN10/RB10)-->(CON6/Pin28) drive state low */
	LATBbits.LATB10 = 0;
	/* set pin (AN10/RB10)-->(CON6/Pin28) as output */
	TRISBbits.TRISB10 = 0;
}

/******************************************************************************
 * Function:	ADC1_config()
 * Description:	Configures ADC1
 ******************************************************************************/
void ADC1_config(void)
{
	/* (CON5/Pin28)-(AN23/RA7) Pin as input */
	LATAbits.LATA7 = 0;		/* set LED (AN23/RA7) drive state low */
	TRISAbits.TRISA7 = 1;	/* set LED pin (AN23/RA7) as input */
	/* (CON5/Pin25)-(AN22/RA6) Pin as input */
	LATAbits.LATA6 = 0;	/* set LED (AN22/RA6) drive state low */
	TRISAbits.TRISA6 = 1;	/* set LED pin (AN22/RA6) as input */

	/*ADC Configuration*/
	AD1PCFGL = 0xFFFF;		//ADC1 Port Configuration Register Low
	AD1PCFGH = 0xFFFF;		//ADC1 Port Configuration Register High
	//AD1PCFGLbits.PCFG13 = 0;	// AN13 is Analog Input
	AD1PCFGHbits.PCFG23 = 0;
	AD1PCFGHbits.PCFG22 = 0;
	AD1CON2bits.VCFG = 0;   //Converter Voltage Reference Configuration bits
							//(ADRef+=AVdd, ADRef-=AVss)
	AD1CON3bits.ADCS = 63;//10;	// ADC Conversion Clock Select bits
							    //(Tad = Tcy*(ADCS+1) = (1/40000000)*64 = 1.6us)
						        //Tcy=Instruction Cycle Time=40MIPS
	AD1CON2bits.CHPS = 0;	// Selects Channels Utilized bits,
						    //When AD12B = 1, CHPS<1:0> is: U-0, Unimplemented,
							//Read as ‘0’
	AD1CON1bits.SSRC = 7;	/*Sample Clock Source Select bits:
							  111 = Internal counter ends sampling and starts
							  conversion (auto-convert)
							  110 = Reserved
							  101 = Reserved
							  100 = Reserved
							  011 = MPWM interval ends sampling and starts
							  conversion
							  010 = GP timer (Timer3 for ADC1, Timer5 for ADC2)
							  compare ends sampling and starts conversion
							  001 = Active transition on INTx pin ends sampling
							  and starts conversion
							  000 = Clearing sample bit ends sampling and starts
							  conversion*/
	AD1CON3bits.SAMC = 31;//0;	// Auto Sample Time bits. (31*Tad = 49.6us)
	AD1CON1bits.FORM = 0;	// Data Output Format bits. Integer
							/* For 12-bit operation:
							   11 = Signed fractional (DOUT = sddd dddd dddd
							   0000, where s = .NOT.d<11>)
							   10 = Fractional (DOUT = dddd dddd dddd 0000)
							   01 = Signed Integer (DOUT = ssss sddd dddd dddd,
							   where s = .NOT.d<11>)
							   00 = Integer (DOUT = 0000 dddd dddd dddd)*/
	AD1CON1bits.AD12B = 1;	/* Operation Mode bit:
							   0 = 10 bit
							   1 = 12 bit*/
	AD1CON1bits.ASAM  = 0;	/* ADC Sample Auto-Start bit:
						       1 = Sampling begins immediately after last
						       conversion. SAMP bit is auto-set.
							   0 = Sampling begins when SAMP bit is set*/
	AD1CHS0bits.CH0SA = 23;	// MUXA +Ve input selection (AIN23) for CH0.
	AD1CHS0bits.CH0SA = 22;	// MUXA +Ve input selection (AIN22) for CH0.
	AD1CHS0bits.CH0NA = 0;	// MUXA  -Ve input selection (Vref-) for CH0.
	AD1CON1bits.ADON  = 1;	/* ADC Operating Mode bit.
							 * Turn on the A/D converter */
}


/******************************************************************************
 * Function:	PWM_config()
 * Description:	Configures PWM actuator
 ******************************************************************************/
void PWM_config(void)
{
	PTCONbits.PTMOD = 0;	/*PWM Time Base Mode Select bits
							  11 =PWM time base operates in a Continuous
							  Up/Down Count mode with interrupts for double
							  PWM updates
							  10 =PWM time base operates in a Continuous
							  Up/Down Count mode
							  01 =PWM time base operates in Single Pulse mode
							  00 =PWM time base operates in a Free-Running
							  mode*/

	PTPER = 0x3FFF;			/* PWM Time Base Period Value bits, PWM period
							selection */
	PWMCON1 = 0x0111;		// PWM Control Register 1

	PWMCON2bits.IUE = 0;	// Immediate update period enable
	PWMCON2bits.UDIS = 0;	/* PWM Update Disable bit
							   1 = Updates from Duty Cycle and Period Buffer
							   registers are disabled
							   0 = Updates from Duty Cycle and Period Buffer
							   registers are enabled*/

	OVDCON = 0xff00;		// Override Control Register
	PDC1 = 0x0000;			// Initial PWM value

	PTCONbits.PTEN = 1;		// Enable PWM.
}




/******************************************************************************
 * Function:	EE_Flex_setup()
 * Description:	Configures system clock and initialize devices
******************************************************************************/
void Sys_init(void)
{
	/* Clock setup for 40MIPS */
	CLKDIVbits.DOZEN   = 0;
	CLKDIVbits.PLLPRE  = 0;
	CLKDIVbits.PLLPOST = 0;
	PLLFBDbits.PLLDIV  = 78;

	/* Wait for PLL to lock */
	while(OSCCONbits.LOCK!=1);

	/* Program Timer 1 to raise interrupts */
	T1_program();

  	/* Time init (EDF scheduler) */
  	EE_time_init();

//	/* Configures the Analog to Digital Converter 1.
//	 * Pins: (CON5/Pin28)-(AN23/RA7)
//	 *       (CON5/Pin25)-(AN22/RA6) */
//	ADC1_config();

//	/*Configure the PWM 1 */
//	PWM_config();

	/*Configure the orange led of the FLEX FULL */
	Led_config();

	/*Configure oscilloscope signals */
	Signals_config();

//	/*Configure UART1 in DMA mode */
//	UART1_DMA_config();

	/*Configure enhanced CAN 1 */
	eCAN1_config();
}

