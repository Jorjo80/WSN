#include <ADuC841.h>
#include <stdio.h>

#define CALIB_TIME 5
#define INTR_TIME 1

/**************** FPGA Communication Port: ********************/
sbit TrigByte1N  = P0^7;		   
sbit Ack1N       = P0^6;          
sbit TrigByte2N  = P0^5;
sbit Ack2N       = P0^4;
sbit SelTrigger	 = P0^3;
sbit reset_fpga  = P0^0;
/**************************************************************/

unsigned char DATA_L;
unsigned char DATA_H;
unsigned int datain;

unsigned char flag, c;
unsigned char calib_cnt;
 
unsigned int result,Temp,Hum, LDR; 

unsigned int raw_data[2], calib[2]; //Data from Accelerometer

unsigned int timeout_x=0;//Monitoring Counter X
unsigned int timeout_y=0;//Monitoring Counter Y
unsigned int limit_x=3;//Monitoring Time X
unsigned int limit_y=5;//Monitoring Time Y

unsigned char flagWait, RX_flag, RY_flag;
unsigned char charWait;
unsigned char flagInterrupt=0;

unsigned char RX_Buffer[20];
unsigned char calibrated=0;

/***************** Timer Configuration: **************************/
void _WS_Timer_Config(char value)
{
   	IEIP2 	= 0xA4; // TIC Interruption enable
	SEC 	= 0x00;
	HTHSEC	= 0x00;
	MIN		= 0x00;
	HOUR	= 0x00;
	INTVAL	= value;	//**(Config.)	
	TIMECON	= 0x53; 	// The timer interrupt each second **(Config.)	/* 0x43 = 1/128 seconds */
}	
/*****************************************************************/

/***************** ADC Configuration: ****************************/
void _WS_ADC_Config (void)
{
  	ADCCON1  = 0xAC;     // ADCCON1: ADC Configuration: 12 clock periods for each conversion.                                                                     
	ADCCON2  = 0x03;     // Selects channel 3 & on demand conversion.  (LDR is connected to the ADC3)
}
/*****************************************************************/


/***************** UART configuration: ***************************/
void _WSN_UART841_config()
{
	SCON = 0x52;//SCON: UART Serial Port Control Register	=> Mode 1: 8-bit UART, variable baud rate
	PCON = 0x80;//PCON: power-saving options and general-purpose status flags => SMOD=1 (Double UART Baud Rate)
	
	TMOD = 0x21;//Timer 1 Set M1 for 8-bit autoreload timer, Timer 0 Set M0 16-bit 
	TH1  = 0xDC;// 19200 ADuC841        //TH1 holds a value which is to be reloaded into TL1 each time it overflows. (BaudRate = 19200 bps)
	TR1  = 1;   //Start timer 1

	TI  = 1;   //1 bit1(SCON): Serial Port Transmit Interrupt Flag.
	ES  = 1;	// Serial Port interruption enable
	ET1 = 0;	// Timer 1 Interruption Disable	

	PS = 1;

	EA  = 1; 	// Global Enable Interruption Flag
}
/****************************************************************/

/****************** ADC Conversion: *****************************/
int _WSN_ADC_conversion()
{
	unsigned int sensorData;

	//*** Sigle conversion:
	SCONV = 1;
	while (SCONV == 1);

	sensorData = ((ADCDATAH & 0x0F) * 0x0100) + ADCDATAL;
	
	SCONV = 0; // Conversion Flag
	
	return (sensorData);

}
/*****************************************************************/

/****************** FPGA Initial config. *************************/
void _WSN_ini_FPGA(void)
{ 
 	   TrigByte1N  = 1;		   
	   TrigByte2N  = 1;
	   SelTrigger  = 0;
	   reset_fpga  = 1;
}

/****************** FPGA-DATA capture: **************************/
int _WSN_FPGA(bit sensorSelector)
{     
   unsigned int fpga_data;
         
   SelTrigger  = sensorSelector;
   TrigByte1N = 0; 
   while (Ack1N == 1){};
   
   DATA_L = P2;    	   // LSB
   TrigByte1N = 1;	   // Release Trigger1
    
   TrigByte2N = 0;	   //Trigger second data byte
   while (Ack2N == 1){};
   
   DATA_H = P2; 	  // MSB
   TrigByte2N = 1;    // Release Trigger2

   fpga_data = DATA_L + 256*(int)DATA_H;

   return(fpga_data);  
   
}
/*****************************************************************/

/**************** Serial Transmission: ***************************/
 
void _WSN_Write_UART(char *message)
{  
  do{
	TI = 0;
  	SBUF = *message++;
	while (!TI);
  }while(*message != '\0'); // wait untin null character is read from the TX message
    TI = 0;
}		   

/**************** Serial Reception: ******************************/
void _WSN_Read_UART(char *message)
{  
  do{
	RI = 0;
	while (!RI);
  	*message++ = SBUF;	
  }while(SBUF != '\r');	 // wait untin null character is read from the RX message
  *message = '\0'; // Ending writing a null character into the buffer
}																	
		   
/***************** Serial Interruption: **************************/
  
void _CEI_Serial_interrupt(void) interrupt 4 using 0
{
 	ES = 0;	// Disable Serial Interruption

	// Data Transmision:--------------------
	 //if (TI == 1)   TI = 0;

	//Data Reception: ----------------------
	if (RI == 1) {
	   
	   if(/*!flagWait && */SBUF == 'w'){	// If we are not waiting for a particular character, this condition can be removed, so that every received byte will be stored in RX_Buffer	
		  _WSN_Read_UART(RX_Buffer);
		  // retransmiting the message just for testing:
		  //_WSN_Write_UART(RX_Buffer);
		  RX_flag = 1;
	   }
		else if(/*!flagWait && */SBUF == 'z'){	// If we are not waiting for a particular character, this condition can be removed, so that every received byte will be stored in RX_Buffer	
		  _WSN_Read_UART(RX_Buffer);
		  // retransmiting the message just for testing:
		  //_WSN_Write_UART(RX_Buffer);
		  RY_flag = 1;
	   }

	   else if(flagWait == 1 && SBUF == charWait){ // Condition for waiting an answer prompt, such as 'O' for "OK", etc. 
		  flagWait = 0;
	   }
	 RI=0;
	}//-------------------------------------

	ES = 1; // Esable Serial Interruption
}		 
/*****************************************************************/


/***************** Sensors reading functionalities: ***************/
void _WSN_sensors_reading(void){

   int i;  //result
	
	/************ Y AXIS: ******************/
   	raw_data[0] = _WSN_FPGA(0); /* = Temperature, ACC Y **/
	for(i=0;i<100;i++);
	/*************************************/

	/************ X AXIS: ******************/
	raw_data[1] = _WSN_FPGA(1); /* = Humidity, ACC X **/
	/*************************************/
	
}

 /***************** Sensors reading functionalities: ***************/
void _WSN_sensors_x(void){
	int i=0;
	raw_data[1] = _WSN_FPGA(1); /* = Humidity, ACC X **/
	for(i=0;i<100;i++);
}

void _WSN_sensors_y(void){
	int i=0;
	raw_data[0] = _WSN_FPGA(0); /* = Humidity, ACC Y **/
	for(i=0;i<100;i++);
}
/*****************************************************************/


/*****************************************************************/

/****************** ZigBee read: *********************************/
/** ASCII  = Value of the character to wait.
/** getsmj = It allows to get caracters from the serial port and 
/** print them until ASCII arrives. 
**/

void _WSN_wait_answer(char ASCII,char getmsj)
{  
	charWait = ASCII;	
	flagWait = 1;	 
	while(flagWait);
	flagWait = 1;
}

/*****************************************************************/


/**************** ZigBee Configuration: ************************/

void _WSN_ZigBee_config(char type)
{ 		
	_WSN_Write_UART("AT&F\n\r\0");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATS00=0040\n\r\0");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATS02=0007\n\r\0");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATS03=1111111111111117\n\r\0");
	_WSN_wait_answer('O',0);
	//printf("AT+DASSL\n");
	//_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATZ\n\r\0");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("AT+JN\n\r\0");
	_WSN_wait_answer('J',0);
}
/***************** Timer Interruption: ***************************/
void _WSN_interrupt_TimeInterval() interrupt 10 using 3 
{ 
   //unsigned int result,Temp,Hum,LDR,Axis;

      /** DO NOT EDIT *********/
	c++;
	if (c==2){
	    reset_fpga = 0;
		reset_fpga = 1;
		c = 0;
	}   
	  /************************/
	 
	 if(calibrated){
		if(timeout_x>=limit_x){
			char message_x[30];
			sprintf(message_x, "AT+UCAST:0000,X: %d \n\r\0", raw_data[1]-calib[1]);
		   _WSN_sensors_x();
		   _WSN_Write_UART(message_x);
		   _WSN_wait_answer('A',0);
		   timeout_x=0;
		}
		else  timeout_x++;
	
	 	if(timeout_y>=limit_y){
			char message_y[30];
			sprintf(message_y, "AT+UCAST:0000,Y: %d \n\r\0", raw_data[0]-calib[0]);
		   _WSN_sensors_y();
		   _WSN_Write_UART(message_y);
		   _WSN_wait_answer('A',0);
		   timeout_y=0;
		}
		else  timeout_y++;
  	 }
    flag = 1;
      
}
/*****************************************************************/

/******************* Message Detection: *************************/
void _WSN_message_detect()
{  
 	_WSN_wait_answer('U',0);
	_WSN_wait_answer(':',0);
	_WSN_wait_answer(',',1);
	 putchar('\t');
	_WSN_wait_answer('=',0);
	_WSN_wait_answer(0x03,1); 
}



 
/******************* Main Function: *****************************/
void main()
{
	//---- Peripheral Configurations: -------------
	_WSN_UART841_config();
	_WSN_ini_FPGA();
	//printf ("Connected\n\r");
	_WSN_ZigBee_config('o');  
	_WS_Timer_Config(INTR_TIME);
	//printf ("ZigBee Config\n\r");

	calibrated=0;
	flagWait = 0;
	RX_flag = 0;
	RY_flag=0;

	//printf ("Calibrating\n\r");
	calib_cnt=0;
	flag = 0;//Interruption flag
	while(calib_cnt < CALIB_TIME){
		if (flag == 1){
		  	calib_cnt++;
			_WSN_sensors_reading();//Force update of Accelerometer
	   		calib[0]+=raw_data[0]; //Y Axis
	   		calib[1]+=raw_data[1]; //X Axis
		  	flag =0;
		}  
	}
	calib_cnt=0;
	calib[0]=calib[0]/CALIB_TIME;
	calib[1]=calib[1]/CALIB_TIME;	 
	//printf ("End of calibration\n\r");			
	calibrated=1;
	
	// --------------------------------------------
	while (1){
		if(RX_flag == 1){
			sscanf (RX_Buffer,"%d",&limit_x); 
		 	RX_flag = 0;
		}	
		if(RY_flag == 1){
			sscanf (RX_Buffer,"%d",&limit_y); 
		 	RY_flag = 0;
		}		   		  
	}
}
/****************************************************************/
