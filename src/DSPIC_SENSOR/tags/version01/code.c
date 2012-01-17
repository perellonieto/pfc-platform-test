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
 *****************************************************************************/

/******************************************************************************
 *
 *  SSSS   AAA   M   M  PPPP   L      EEEEE  RRRR           AAA    CCCC  TTTTT
 * S      A   A  MM MM  P   P  L      E      R   R         A   A  C        T
 *  SSS   AAAAA  M M M  PPPP   L      EEE    RRRR  ======= AAAAA  C        T
 *     S  A   A  M   M  P      L      E      R  R          A   A  C        T
 * SSSS   A   A  M   M  P      LLLLL  EEEEE  R   R         A   A   CCCC    T
 *
 *****************************************************************************/

/******************************************************************************
 * SAMPLER-ACTUATOR BOARD
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
//unsigned long ID_PLANT=1;

// MESSAGE id in rest of application
#define  REFERENCE_MESSAGE 			(0)
#define  SUPERVISOR_MESSAGE 		(1)

// MESSAGE ID
#define  ID_PLANT					(1<<2)

// MESSAGE PRIORITY
#define  GENERAL_PRIORITY			(0<<10)

// MESSAGE CLASSE
#define  CONTROL_MESSAGE 			((unsigned long)0<<26)
#define  GRANTED_SENSOR_MESSAGE 	((unsigned long)1<<26)
#define  GENERAL_PURPOSE_MESSAGE 	((unsigned long)2<<26)
#define  BEST_EFFORT_SENSOR_MESSAGE ((unsigned long)3<<26)

unsigned long ID_TO_ACTUATOR 	= CONTROL_MESSAGE | GENERAL_PRIORITY | ID_PLANT;
unsigned long ID_FROM_SENSOR	= GRANTED_SENSOR_MESSAGE | GENERAL_PRIORITY | ID_PLANT;
unsigned long ID_REFERENCE		= GENERAL_PURPOSE_MESSAGE | GENERAL_PRIORITY | ID_PLANT | REFERENCE_MESSAGE;
unsigned long ID_SUPERVISOR		= GENERAL_PURPOSE_MESSAGE | GENERAL_PRIORITY | ID_PLANT | SUPERVISOR_MESSAGE;


static float v_max=3.3;  //dsPIC voltage reference
static float offset=0.0;//0.06;//OPAMP offset

static float r=-0.5;
static float x[2]={0,0};
static float u=0;



// Define ECAN Message Buffers
ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma),aligned(ECAN1_MSG_BUF_LENGTH*16)));
//ECAN2MSGBUF ecan2msgBuf __attribute__((space(dma),aligned(ECAN2_MSG_BUF_LENGTH*16)));
// CAN Messages in RAM
//mID rx_ecan1message1;//RX Sensor to controller message
mID rx_ecan1message2;//RX Controller to actuator message
mID rx_ecan1message3;//RX Controller updates reference (supervision)
//mID rx_ecan1message4;//RX Not used
mID tx_ecan1message; //TX Transmission message

void Send_Sensor2Controller_message(float *);//Function prototypes

/* Read RCRC circuit output voltages */
void Read_State(void)
{
	AD1CHS0 = 22; 		   // Input ADC channel selection
	AD1CON1bits.SAMP = 1;  // Start conversion
	while(!IFS0bits.AD1IF);// Wait till the EOC
	IFS0bits.AD1IF = 0;    // Reset ADC interrupt flag
	x[0]=(ADC1BUF0*v_max/4096)-v_max/2-offset;// Acquire data and scale

	AD1CHS0 = 23; 		   // Input ADC channel selection
	AD1CON1bits.SAMP = 1;  // Start conversion
	while(!IFS0bits.AD1IF);// Wait till the EOC
	IFS0bits.AD1IF = 0;    // Reset ADC interrupt flag
	x[1]=(ADC1BUF0*v_max/4096)-v_max/2-offset;// Acquire data and scale
}

/* Sensor Task */

TASK(TaskSensor)
{
	LATBbits.LATB14 ^= 1;//Toggle orange led

	Read_State();

	Send_Sensor2Controller_message(&x[0]);//identifier=ID_PLANT
}



/* Actuator Task */
static float *p_u = (float *)&rx_ecan1message2.data[0];

TASK(TaskActuator)
{
	u=(*p_u);

	PDC1=((*(p_u))/v_max)*0x7fff+0x3FFF;
}


/* Send data using the UART port 1 via RS-232 to the PC
 * Note: This Task takes less than 0.0002 seconds */

TASK(TaskSupervision)
{
	static unsigned char *p_r = (unsigned char *)&r;
	static unsigned char *p_x0= (unsigned char *)&x[0];
	static unsigned char *p_x1= (unsigned char *)&x[1];
	static unsigned char *p_u = (unsigned char *)&u;

	static unsigned long sys_time=0;

	LATBbits.LATB10 = 1; //To get time with the oscilloscope

	sys_time=GetTime();  //Get system time (EDF Scheduler)

	Read_State();        //Before sending state, read it

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

	//Force sending data
	DMA4CONbits.CHEN  = 1;			// Re-enable DMA4 Channel
	DMA4REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer

	LATBbits.LATB10 = 0; //To get time with the oscilloscope
}

/* main function, only to initialize software and hardware,
 * fire alarms, and implement background activities */
int main(void)
{
	Sys_init();//Initialize clock, devices and periphericals

	SetRelAlarm(AlarmSensor,1000,10);//Sensor activates every 50ms
	SetRelAlarm(AlarmSupervision, 1000, 10);//Data is sent to the PC every 10ms

	/* Forever loop: background activities (if any) should go here */
	for (;;);

	return 0;
}


static unsigned char *p_data= NULL;
void Send_Sensor2Controller_message(float *data)
{

	p_data=(unsigned char *)data;
	C1CTRL1bits.ABAT = 1;
	while(C1TR01CONbits.TXREQ0){};

	tx_ecan1message.buffer=0;//Buffer number
	tx_ecan1message.frame_type=1;//0->Std Id, 1->Ext Id

	tx_ecan1message.id=ID_FROM_SENSOR;//Identifier;
	tx_ecan1message.message_type=0;//0->Normal, 1->Remote Transmit
	tx_ecan1message.data_length=8;//Length of data (0 to 8 bytes)
	tx_ecan1message.data[0]= *p_data;
	tx_ecan1message.data[1]=*(p_data+1);
	tx_ecan1message.data[2]=*(p_data+2);
	tx_ecan1message.data[3]=*(p_data+3);
	tx_ecan1message.data[4]=*(p_data+4);
	tx_ecan1message.data[5]=*(p_data+5);
	tx_ecan1message.data[6]=*(p_data+6);
	tx_ecan1message.data[7]=*(p_data+7);
	ecan1SendMessage(&tx_ecan1message);

	while(C1TR01CONbits.TXREQ0){};
}




/* CAN bus 1 Interrupt, ISR2 type */
ISR2(_C1Interrupt)
{
	IFS2bits.C1IF = 0; // clear interrupt flag

	/* Transmission interrupt (nothing to be done but clear flag) */
	if(C1INTFbits.TBIF)
    {
    	C1INTFbits.TBIF = 0;
    }

	/*Reception interrupt, different code for different filtered id's */
    if(C1INTFbits.RBIF)
    {
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
			ActivateTask(TaskActuator);
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
	}
}
