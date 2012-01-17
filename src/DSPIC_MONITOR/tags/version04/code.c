/******************************************************************************
 *  FILE		 	: code.c
 *  DESCRIPTION  	: main program
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
#include "code.h"

void Send_buffer_to_pc()
{
	int i, iterations;
	static unsigned long sys_time=0;

	if (ecan1counter_out != ecan1counter_in)
	{
		LATBbits.LATB10 = 1; //To get time with the oscilloscope

		sys_time=GetTime();  //Get system time (EDF Scheduler)

		OutBuffer[0]=0x01;//header;
		//OutBuffer[1]=(unsigned char)(sys_time>>24);//4th byte of unsigned long
		//OutBuffer[2]=(unsigned char)(sys_time>>16);//3rd byte of unsigned long
		//OutBuffer[3]=(unsigned char)(sys_time>>8); //2nd byte of unsigned long
		//OutBuffer[4]=(unsigned char)sys_time;      //1st byte of unsigned long

		iterations = 0;
		while (ecan1counter_out != ecan1counter_in)
		{
			for (i = 0; i < INPUT_DATA_SIZE; i++)
			{
				//OutBuffer[((i+1)*iterations)+1] = rx_ecan1message[ecan1counter_out].data[i];
				//debug
				OutBuffer[(i+iterations*(INPUT_DATA_SIZE+1))+1] = 0;
			}
			// debug
			OutBuffer[(iterations*(INPUT_DATA_SIZE+1))+1] = (short) (rx_ecan1message[ecan1counter_out].id+1);
			ecan1counter_out = (ecan1counter_out+1)%MAX_ECAN1_RX_MESSAGES;
			iterations++;
		}

		//Force sending data
		DMA4CONbits.CHEN  = 1;			// Re-enable DMA4 Channel
		DMA4REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer

		LATBbits.LATB10 = 0; //To get time with the oscilloscope
	}
}


/* Send data using the UART port 1 via RS-232 to the PC
 * Note: This Task takes less than 0.0002 seconds */
/* maikel note: in this moment this time is false */

TASK(TaskSupervision)
{
	Send_buffer_to_pc();
}

TASK(TaskToggleLed)
{
	LATBbits.LATB14 ^= 1;//Toggle orange led
}

/* main function, only to initialize software and hardware,
 * fire alarms, and implement background activities */
int main(void)
{
	Sys_init();//Initialize clock, devices and periphericals

	//SetRelAlarm(AlarmSupervision, 1000, 10);//Data is sent to the PC every 10ms
	//SetRelAlarm(AlarmToggleLed, 1000, 500); // Led is toggled.

	/* Forever loop: background activities (if any) should go here */
	for (;;);

	return 0;
}

/*
 * @param id 			identifier to put in can message
 * @param data 			message to send
 * @param data_length	length of the message ( must be < ECAN1_MSG_BUF_LENGTH )
 */
void Send_CAN_message(int id, float *data, int data_length)
{
	int i;
	p_data=(unsigned char *)data;
	C1CTRL1bits.ABAT = 1;
	while(C1TR01CONbits.TXREQ0){};

	tx_ecan1message.buffer=0;//Buffer number
	tx_ecan1message.frame_type=1;//0->Std Id, 1->Ext Id

	tx_ecan1message.id=id;//Identifier;
	tx_ecan1message.message_type=0;//0->Normal, 1->Remote Transmit
	tx_ecan1message.data_length=data_length;

	for (i = 0; i < data_length; i++)
	{
		tx_ecan1message.data[i]=*(p_data+i);
	}

	ecan1SendMessage(&tx_ecan1message);



	while(C1TR01CONbits.TXREQ0){};
}


/* CAN bus 1 Interrupt, ISR2 type */
ISR2(_C1Interrupt)
{
	IFS2bits.C1IF = 0; // clear interrupt flag

	// Transmission interrupt (nothing to be done but clear flag)
	if(C1INTFbits.TBIF)
    {
    	C1INTFbits.TBIF = 0;
    }
/*
	//Reception interrupt, different code for different filtered id's
    if(C1INTFbits.RBIF)
    {



		//Filter 2(id=ID_PLANT+1): Controller to Actuator message
		if(C1RXFUL1bits.RXFUL1==1)
	    {
			//Tells rxECAN1 the buffer to pass from DMA to RAM
	    	rx_ecan1message[ecan1counter_in].buffer=1;

	    	C1RXFUL1bits.RXFUL1=0;
		    rxECAN1(&rx_ecan1message[ecan1counter_in]);
			C1INTFbits.RBIF = 0;

			ecan1counter_in = (ecan1counter_in+1)%MAX_ECAN1_RX_MESSAGES;
	    }
		//Filter 2(id=ID_PLANT+1): Controller to Actuator message
		else if(C1RXFUL1bits.RXFUL2==1)
	    {
			//Tells rxECAN1 the buffer to pass from DMA to RAM
	    	rx_ecan1message[ecan1counter_in].buffer=2;

	    	C1RXFUL1bits.RXFUL2=0;
		    rxECAN1(&rx_ecan1message[ecan1counter_in]);
			C1INTFbits.RBIF = 0;

			ecan1counter_in = (ecan1counter_in+1)%MAX_ECAN1_RX_MESSAGES;
	    }
		//Filter 2(id=ID_PLANT+1): Controller to Actuator message
		else if(C1RXFUL1bits.RXFUL3==1)
			{
				//Tells rxECAN1 the buffer to pass from DMA to RAM
				rx_ecan1message[ecan1counter_in].buffer=3;

				C1RXFUL1bits.RXFUL3=0;
				rxECAN1(&rx_ecan1message[ecan1counter_in]);
				C1INTFbits.RBIF = 0;

				ecan1counter_in = (ecan1counter_in+1)%MAX_ECAN1_RX_MESSAGES;
			}

		if (ecan1counter_in == ecan1counter_out-1) Send_buffer_to_pc();
	}*/
}

ISR2(_DMA5Interrupt) //NOTE: Disabled when IEC3bits.DMA5IE = 0;
{
	int i;
	float *p_k0=(float *)&InBufferA[0];

	for (i = 0; i < INPUT_DATA_SIZE; i++) OutBuffer[i] = InBufferA[i];
	//for (i = INPUT_DATA_SIZE; i < OUTPUT_DATA_SIZE; i++) OutBuffer[i] = 0;

	/*Code to update received data*/
	k_updated[0]= *(p_k0);
	k_updated[1]= *(p_k0+1);

	DMA4CONbits.CHEN  = 1;			// Re-enable DMA4 Channel
	DMA4REQbits.FORCE = 1;

	//LATBbits.LATB14 = !LATBbits.LATB14;
	switch (InBufferA[0])
	{
	case ID_PERCENT:
		SetRelAlarm(AlarmToggleLed, 1000,(InBufferA[7]+1)*5);
		break;
	default:
		break;
	}

	IFS3bits.DMA5IF = 0; // Clear the DMA1 Interrupt Flag
}
