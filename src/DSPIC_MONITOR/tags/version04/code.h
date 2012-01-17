/******************************************************************************
 *  FILE		 	: code.h
 *  DESCRIPTION  	: main headers
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
 *
 *   REVISION HISTORY:
 *			 VERSION: 0.2
 *     		  AUTHOR: Miquel Perelló Nieto
 * 				DATE: October 2011
 * 			COMMENTS:
 *****************************************************************************/

#ifndef CODE_H_
#define CODE_H_

/******************************************************************************
 *   __  __  ____  _   _ _____ _______ ____  _____
 *	|  \/  |/ __ \| \ | |_   _|__   __/ __ \|  __ \
 *	| \  / | |  | |  \| | | |    | | | |  | | |__) |
 *	| |\/| | |  | | . ` | | |    | | | |  | |  _  /
 *	| |  | | |__| | |\  |_| |_   | | | |__| | | \ \
 *	|_|  |_|\____/|_| \_|_____|  |_|  \____/|_|  \_\
 *
 *****************************************************************************/

/******************************************************************************
 * MONITOR BOARD
 *****************************************************************************/

/******************************************************************************
 * Identifiers:
 * ID=ID_PLANT-->Sensor to Controller message
 * ID=ID_PLANT+1-->Controller to Actuator message
 * ID=ID_PLANT+2-->Reference update
 *****************************************************************************/
#include "ee.h"
#include "cpu/pic30/inc/ee_irqstub.h"
#include "setup.h"
#include "uart_dma.h"
#include "e_can1.h"


_FOSCSEL(FNOSC_PRIPLL);	// Primary (XT, HS, EC) Oscillator with PLL
_FOSC(OSCIOFNC_ON & POSCMD_XT); // OSC2 Pin Function: OSC2 is Clock Output
								// Primary Oscillator Mode: XT Crystal
_FWDT(FWDTEN_OFF);		// Watchdog Timer Enabled/disabled by user software
_FGS(GCP_OFF);			// Disable Code Protection


/*	Variables	*/
unsigned long ID_PLANT=1;

unsigned long ID_SENSOR			= 0;
unsigned long ID_ACTUATOR		= 1;
unsigned long ID_CONTROLATOR	= 2;
unsigned long ID_MONITOR		= 3;

unsigned long ID_LINK		 	= 1;

#define ID_PERCENT		4

static unsigned char *p_data= NULL;

//static float v_max=3.3;  //dsPIC voltage reference
//static float offset=0.0;//0.06;//OPAMP offset

//static float r=-0.5;
//static float x[2]={0,0};
//static float u=0;



// Define ECAN Message Buffers
ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma),aligned(ECAN1_MSG_BUF_LENGTH*16)));
//ECAN2MSGBUF ecan2msgBuf __attribute__((space(dma),aligned(ECAN2_MSG_BUF_LENGTH*16)));

mID tx_ecan1message; //TX Transmission message
// CAN Messages in RAM
#define MAX_ECAN1_RX_MESSAGES 8
mID rx_ecan1message[MAX_ECAN1_RX_MESSAGES];
unsigned char ecan1counter_in = 0;
unsigned char ecan1counter_out = 0;

void Send_buffer_to_pc();

/*
 * @param id 			identifier to put in can message
 * @param data 			message to send
 * @param data_length	length of the message ( must be < ECAN1_MSG_BUF_LENGTH )
 */
void Send_CAN_message(int id, float *data, int data_length);


#endif /* CODE_H_ */
