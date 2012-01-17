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

void Send_CAN_useless()
{
	C1CTRL1bits.ABAT = 1;
	while(C1TR01CONbits.TXREQ0){};

	tx_ecan1message.buffer=0;//Buffer number
	tx_ecan1message.frame_type=1;//0->Std Id, 1->Ext Id

	tx_ecan1message.id=0x00000000;//Identifier;
	tx_ecan1message.message_type=0;//0->Normal, 1->Remote Transmit
	tx_ecan1message.data_length=8;//Length of data (0 to 8 bytes)
	tx_ecan1message.data[0]= 0x0;
	tx_ecan1message.data[1]= 0x0;
	tx_ecan1message.data[2]= 0x0;
	tx_ecan1message.data[3]= 0x0;
	tx_ecan1message.data[4]= 0x0;
	tx_ecan1message.data[5]= 0x0;
	tx_ecan1message.data[6]= 0x0;
	tx_ecan1message.data[7]= 0x0;

	ecan1SendMessage(&tx_ecan1message);

	while(C1TR01CONbits.TXREQ0){};
}

static float *p_x0 = (float *)&rx_ecan1message1.data[0];
static float *p_x1 = (float *)&rx_ecan1message1.data[4];
static float x0=0;
static float x1=0;
TASK(TaskControllerMonitor)
{
	x0=*(p_x0);//Get state x[0] from rx_ecan1_message1[0]..[3] data field
	x1=*(p_x1);//Get state x[1] from rx_ecan1_message1[4]..[7] data field
	x[0] = x0;
	x[1] = x1;
}

/* Actuator Task */
static float *p_u = (float *)&rx_ecan1message2.data[0];
static unsigned long sys_time=0;

TASK(TaskActuatorMonitor)
{
	//sys_time=GetTime();  //Get system time (EDF Scheduler)
	u=(*p_u);
	x0=*(p_x0);//Get state x[0] from rx_ecan1_message1[0]..[3] data field
	x1=*(p_x1);//Get state x[1] from rx_ecan1_message1[4]..[7] data field
}

/* Send data using the UART port 1 via RS-232 to the PC
 * Note: This Task takes less than 0.0002 seconds */
/* maikel note: in this moment this time is false */

TASK(TaskSupervision)
{
	//Send_buffer_to_pc();

	static unsigned char *p_r = (unsigned char *)&r;
	static unsigned char *p_x0= (unsigned char *)&x[0];
	static unsigned char *p_x1= (unsigned char *)&x[1];
	static unsigned char *p_u = (unsigned char *)&u;

	//static unsigned long sys_time=0;

	LATBbits.LATB10 = 1; //To get time with the oscilloscope

	sys_time=GetTime();  //Get system time (EDF Scheduler)

	//Read_State();        //Before sending state, read it

	OutBuffer[0]=0x01;//header;
	OutBuffer[1]=(unsigned char)(sys_time>>24);//4th byte of unsigned long
	OutBuffer[2]=(unsigned char)(sys_time>>16);//3rd byte of unsigned long
	OutBuffer[3]=(unsigned char)(sys_time>>8); //2nd byte of unsigned long
	OutBuffer[4]=(unsigned char)sys_time;      //1st byte of unsigned long
	OutBuffer[5]=*p_r;    //4th byte of float (32bits)
	OutBuffer[6]=*(p_r+1);//3rd byte of float (32bits)
	OutBuffer[7]=*(p_r+2);//2nd byte of float (32bits)
	OutBuffer[8]=*(p_r+3);//1st byte of float (32bits)
	OutBuffer[9]=*p_x0;
	OutBuffer[10]=*(p_x0+1);
	OutBuffer[11]=*(p_x0+2);
	OutBuffer[12]=*(p_x0+3);
	OutBuffer[13]=*p_x1;
	OutBuffer[14]=*(p_x1+1);
	OutBuffer[15]=*(p_x1+2);
	OutBuffer[16]=*(p_x1+3);
	OutBuffer[17]=*p_u;
	OutBuffer[18]=*(p_u+1);
	OutBuffer[19]=*(p_u+2);
	OutBuffer[20]=*(p_u+3);

	// This is for new information
	OutBuffer[22]=0x00;//number of devices;
	unsigned long id;
	int i = 23;
	while (get_device(&id))
	{
		OutBuffer[i++]=(unsigned char)(id>>24);	//4th byte of unsigned long
		OutBuffer[i++]=(unsigned char)(id>>16);	//3rd byte of unsigned long
		OutBuffer[i++]=(unsigned char)(id>>8); 	//2nd byte of unsigned long
		OutBuffer[i++]=(unsigned char)id;   //1st byte of unsigned long
		OutBuffer[22]++;
	}
	if (OutBuffer[22] != 0x00)
		OutBuffer[21]=0x01;//header;

	//Force sending data
	DMA4CONbits.CHEN  = 1;			// Re-enable DMA4 Channel
	DMA4REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer

	LATBbits.LATB10 = 0; //To get time with the oscilloscope
}

TASK(TaskToggleLed)
{
	LATBbits.LATB14 ^= 1;//Toggle orange led
}


TASK(TaskCANUseless)
{
	Send_CAN_useless();
}

/* main function, only to initialize software and hardware,
 * fire alarms, and implement background activities */
int main(void)
{
	Sys_init();//Initialize clock, devices and periphericals

	init_devices_list();

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

	/*Reception interrupt, different code for different filtered id's */
	if(C1INTFbits.RBIF)
	{
		LATBbits.LATB14 ^= 1;//Toggle orange led
    	//Filter 1(id=ID_PLANT): Sensor to controller message
    	if(C1RXFUL1bits.RXFUL1==1)
	    {
    		/* Tells rxECAN1 the buffer to pass from DMA to RAM */
	    	rx_ecan1message1.buffer=1;

	    	C1RXFUL1bits.RXFUL1=0;
		    rxECAN1(&rx_ecan1message1);
			C1INTFbits.RBIF = 0;

			/* Custom code to address Sensor message */
		    ActivateTask(TaskControllerMonitor);
	    }

		//Filter 2(id=ID_PLANT+1): Controller to Actuator message
		if(C1RXFUL1bits.RXFUL2==1)
		{
			/*Tells rxECAN1 the buffer to pass from DMA to RAM */
			rx_ecan1message2.buffer=2;

			C1RXFUL1bits.RXFUL2=0;
			rxECAN1(&rx_ecan1message2);
			C1INTFbits.RBIF = 0;

			/* Custom code to address Controller-->Actuator message
			 * To manage Input/Output delay a SetRelAlarm is preferred */
			ActivateTask(TaskActuatorMonitor);
			//SetRelAlarm(AlarmActuator,50-tp,0);
		}

		//Filter 3(id=ID_PLANT+2): Controller updated reference (supervision)
		if(C1RXFUL1bits.RXFUL3==1)
		{
			/*Tells rxECAN1 the buffer to pass from DMA to RAM */
			rx_ecan1message3.buffer=3;

			C1RXFUL1bits.RXFUL3=0;
			rxECAN1(&rx_ecan1message3);
			C1INTFbits.RBIF = 0;

			/* Custom code to address the Controller message which updates
			 * the reference change */
			r=*(float *)(&rx_ecan1message3.data[0]);
		}

		//Filter 4(id=ALL): all identifiers
		if(C1RXFUL1bits.RXFUL4==1)
		{
			/*Tells rxECAN1 the buffer to pass from DMA to RAM */
			rx_ecan1message4.buffer=4;

			C1RXFUL1bits.RXFUL4=0;
			rxECAN1(&rx_ecan1message4);
			C1INTFbits.RBIF = 0;

			/* Custom code to address the Controller message which updates
			 * the reference change */
			if (!search_device(rx_ecan1message4.id))
				add_device(rx_ecan1message4.id);
		}
	}
}


// Interrupcio per escriptura en el buffer d'entrada del port Serie
ISR2(_DMA5Interrupt) //NOTE: Disabled when IEC3bits.DMA5IE = 0;
{
	float *p_k0=(float *)&InBufferA[0];
	unsigned long id_link;

	//for (i = 0; i < INPUT_DATA_SIZE; i++) OutBuffer[i] = InBufferA[i];
	//for (i = INPUT_DATA_SIZE; i < OUTPUT_DATA_SIZE; i++) OutBuffer[i] = 0;

	/*Code to update received data*/
	k_updated[0]= *(p_k0);
	k_updated[1]= *(p_k0+1);

	//DMA4CONbits.CHEN  = 1;			// Re-enable DMA4 Channel
	//DMA4REQbits.FORCE = 1;

	//LATBbits.LATB14 = !LATBbits.LATB14;
	switch (InBufferA[0])
	{
	case SIGNAL_MONITOR:
		id_link = (unsigned long) InBufferA[4]<<24;
		id_link += (unsigned long) InBufferA[5]<<16;
		id_link += (unsigned long) InBufferA[6]<<8;
		id_link += (unsigned long) InBufferA[7];

		ecan1WriteRxAcptFilter(0x0,id_link,0x1,0x1,0x0);  //id=1 to buffer 1
		ecan1WriteRxAcptFilter(0x1,id_link+1,0x1,0x2,0x0);//id=2 to buffer 2
		ecan1WriteRxAcptFilter(0x2,id_link+2,0x1,0x3,0x0);//id=3 to buffer 3
		//ecan1WriteRxAcptFilter(0x3,99,0x1,0x4,0x0);      //id=99 to buffer 4
		CancelAlarm(AlarmSupervision);
		SetRelAlarm(AlarmSupervision, 1000, 10);
		monitoring = 1;
		break;
	case SIGNAL_PERCENT:
		CancelAlarm(AlarmCANUseless);
		SetRelAlarm(AlarmCANUseless, 1000,101-(InBufferA[7]));
		break;
	case SIGNAL_MONITOR+SIGNAL_STOP:
		//ecan1WriteRxAcptFilter(0x0,0xFFFFFFFF,0x1,0x1,0x0);  //id=1 to buffer 1
		//ecan1WriteRxAcptFilter(0x1,0xFFFFFFFF,0x1,0x2,0x0);//id=2 to buffer 2
		//ecan1WriteRxAcptFilter(0x2,0xFFFFFFFF,0x1,0x3,0x0);//id=3 to buffer 3
		ecan1DisableRXFilter(0x0);
		ecan1DisableRXFilter(0x1);
		ecan1DisableRXFilter(0x2);
		//ecan1WriteRxAcptFilter(0x3,0xFFFFFFFF,0x1,0x4,0x0);      //id=99 to buffer 4
		CancelAlarm(AlarmSupervision);
		LATBbits.LATB14 = 0;
		monitoring = 0;
		break;
	case SIGNAL_PERCENT+SIGNAL_STOP:
		CancelAlarm(AlarmCANUseless);
		break;
	case SIGNAL_DEVICES:
		ecan1WriteRxAcptFilter(0x3,0x00000000,0x1,0x4,0x1);      //id=99 to buffer 4
		SetRelAlarm(AlarmSupervision, 1000, 10);
		break;
	case SIGNAL_DEVICES+SIGNAL_STOP:
		ecan1DisableRXFilter(0x3);
		//ecan1WriteRxAcptFilter(0x3,0xFFFFFFFF,0x1,0x4,0x0);      //id=99 to buffer 4
		if (!monitoring) CancelAlarm(AlarmSupervision);
		LATBbits.LATB14 = 0;
		break;
	default:
		break;
	}

	IFS3bits.DMA5IF = 0; // Clear the DMA1 Interrupt Flag
}
