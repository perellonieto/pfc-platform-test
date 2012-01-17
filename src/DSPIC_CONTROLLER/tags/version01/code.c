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
 *  CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
 * C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
 * C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
 * C      O   O  N  NN    T    R  R   O   O  L      L      E      R  R
 *  CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
 *
 *****************************************************************************/

/******************************************************************************
 * CONTROLLER BOARD
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
//#include "uart_dma.h"
#include "e_can1.h"


_FOSCSEL(FNOSC_PRIPLL);	// Primary (XT, HS, EC) Oscillator with PLL
_FOSC(OSCIOFNC_ON & POSCMD_XT); // OSC2 Pin Function: OSC2 is Clock Output
								// Primary Oscillator Mode: XT Crystal
_FWDT(FWDTEN_OFF);		// Watchdog Timer Enabled/disabled by user software
_FGS(GCP_OFF);			// Disable Code Protection

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

/*	Variables	*/
static float v_max=3.3;  //dsPIC voltage reference
static float r=-0.5;
static float u=0;



/******** TODO Controller gains and tracking matrices ********/
/* static float Nu =??;
static float Nx[2] ={?,?};
static float k[2]={?,?}; */

static float Nu =0.0;
static float Nx[2] ={1.0,0.0};
static float k[2]={0.9834 ,  -0.8931};
static float x_hat[2]={0,0};
static float u_ss=0;

// Define ECAN Message Buffers
ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma),aligned(ECAN1_MSG_BUF_LENGTH*16)));
//ECAN2MSGBUF ecan2msgBuf __attribute__((space(dma),aligned(ECAN2_MSG_BUF_LENGTH*16)));
// CAN Messages in RAM
mID rx_ecan1message1;//RX Sensor to controller message
//mID rx_ecan1message2;//RX Controller to actuator message
//mID rx_ecan1message3;//RX Controller updates reference (supervision)
//mID rx_ecan1message4;//RX Not used
mID tx_ecan1message; //TX Transmission message

//void Send_Sensor2Controller_message(float *);//Function prototypes
void Send_Controller2Actuator_message(float *);
void Send_Controller_ref_message(float *);



/* Change the reference value between -0.5 and 0.5V*/
TASK(TaskReferenceChange)
{
	if (r == -0.5)
	{
		r=0.5;
		LATBbits.LATB14 = 1;//Orange led switch on
	}else{
		r=-0.5;
		LATBbits.LATB14 = 0;//Orange led switch off
	}

	/* Supervision Task is implemented in Sampler/Actuator board, but
	 * Reference Change Task is implemented in Controller board. Therefore,
	 * reference (r) must be sent in a message to be updated before
	 * it is sent to Matlab */
	Send_Controller_ref_message(&r);//identifier=ID_PLANT+2
}




/* Controller Task */

static float *p_x0 = (float *)&rx_ecan1message1.data[0];
static float *p_x1 = (float *)&rx_ecan1message1.data[4];
static float x0=0;
static float x1=0;
TASK(TaskController)
{
	x0=*(p_x0);//Get state x[0] from rx_ecan1_message1[0]..[3] data field
	x1=*(p_x1);//Get state x[1] from rx_ecan1_message1[4]..[7] data field

	//u=r;

	x_hat[0]=x0-r*Nx[0];//Nx, Nu matrices needed for regulation purposes
	x_hat[1]=x1-r*Nx[1];
	u_ss=r*Nu;

	u=-k[0]*x_hat[0]-k[1]*x_hat[1]+u_ss;

	/* Check for saturation */
	if (u>v_max/2) u=v_max/2;
	if (u<-v_max/2) u=-v_max/2;

	Send_Controller2Actuator_message(&u);//identifier=ID_PLANT+1
}





/* main function, only to initialize software and hardware,
 * fire alarms, and implement background activities */
int main(void)
{
	Sys_init();//Initialize clock, devices and periphericals

	SetRelAlarm(AlarmReferenceChange,1000,1000);

	/* Forever loop: background activities (if any) should go here */
	for (;;);

	return 0;
}







static unsigned char *p_data= NULL;
void Send_Controller2Actuator_message(float *data)
{
	p_data=(unsigned char *)data;
	C1CTRL1bits.ABAT = 1;
	while(C1TR01CONbits.TXREQ0){};

	tx_ecan1message.buffer=0;//Buffer number
	tx_ecan1message.frame_type=1;//0->Std Id, 1->Ext Id

	tx_ecan1message.id=ID_TO_ACTUATOR;//Identifier;
	tx_ecan1message.message_type=0;//0->Normal, 1->Remote Transmit
	tx_ecan1message.data_length=4;//Length of data (0 to 8 bytes)
	tx_ecan1message.data[0]= *p_data;
	tx_ecan1message.data[1]=*(p_data+1);
	tx_ecan1message.data[2]=*(p_data+2);
	tx_ecan1message.data[3]=*(p_data+3);

	ecan1SendMessage(&tx_ecan1message);

	while(C1TR01CONbits.TXREQ0){};
}


void Send_Controller_ref_message(float *data)
{
	p_data=(unsigned char *)data;
	C1CTRL1bits.ABAT = 1;
	while(C1TR01CONbits.TXREQ0){};

	tx_ecan1message.buffer=0;//Buffer number
	tx_ecan1message.frame_type=1;//0->Std Id, 1->Ext Id

	tx_ecan1message.id=ID_REFERENCE;//Identifier;
	tx_ecan1message.message_type=0;//0->Normal, 1->Remote Transmit
	tx_ecan1message.data_length=4;//Length of data (0 to 8 bytes)
	tx_ecan1message.data[0]= *p_data;
	tx_ecan1message.data[1]=*(p_data+1);
	tx_ecan1message.data[2]=*(p_data+2);
	tx_ecan1message.data[3]=*(p_data+3);

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
    	//Filter 1(id=ID_PLANT): Sensor to controller message
    	if(C1RXFUL1bits.RXFUL1==1)
	    {
    		/* Tells rxECAN1 the buffer to pass from DMA to RAM */
	    	rx_ecan1message1.buffer=1;

	    	C1RXFUL1bits.RXFUL1=0;
		    rxECAN1(&rx_ecan1message1);
			C1INTFbits.RBIF = 0;

			/* Custom code to address Sensor message */
		    ActivateTask(TaskController);
	    }
	}
}
