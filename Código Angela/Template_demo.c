
#include <ADuC841.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

unsigned char flagWait, RX_flag=0;
unsigned char charWait;
unsigned char flagInterrupt=0;

unsigned char flag, c;

unsigned int result,Temp,Hum, LDR; 

unsigned int resulti[2];
int t=0,h=0,l=0;
int t_T=1, t_H=10, t_L=15;

char RX_Buffer[20];
char Mensaje[60], aux[20];

/***************** Timer Configuration: **************************/
void _WS_Timer_Config(char value)
{
   	IEIP2 	= 0xA4; // TIC Interruption enable
	SEC 	= 0x00;
	HTHSEC	= 0x00;
	MIN		= 0x00;
	HOUR	= 0x00;
	INTVAL	= value;	//**(Config.)	
	TIMECON	= 0x53; 	// The timer interrupt each second **(Config.)	
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

	TI  = 1;   //bit1(SCON): Serial Port Transmit Interrupt Flag.
	ES  = 1;	// Serial Port interruption enable
	ET1 = 0;	// Timer 1 Interruption Disable	

	PS = 1;

	EA  = 1; 	// Global Enable Interruption Flag
	RX_flag = 0;
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
	   if(/*flagWait==0 &&*/ SBUF == 't'){	/** If we are not waiting for a particular character, this condition can be removed, so that
	   					    every received byte will be stored in RX_Buffer	**/
		  _WSN_Read_UART(RX_Buffer);
		  // retransmiting the message just for testing:
		  //_WSN_Write_UART(RX_Buffer);
		  RX_flag = 1;
	   }
	   else if(flagWait == 1 && SBUF == charWait){ /** Condition for waiting an answer prompt, such as 'O' for "OK", etc. **/
		  flagWait = 0;
	   }
	 RI=0;
	}//-------------------------------------

	ES = 1; // Enable Serial Interruption
}
/*****************************************************************/

/***************** Timer Interruption: ***************************/
void _WSN_interrupt_TimeInterval() interrupt 10 using 3 
{ 
   //unsigned int result,Temp,Hum,LDR,Axis;
   flagInterrupt = 1;   
      
}
																	

/*****************************************************************/

/***************** Sensors reading functionalities: ***************/
void _WSN_sensors_reading(void){

   int result[2], i;

   	/** _WSN_FPGA(0) = Temperature, ACC Y **/
	result[0]=_WSN_FPGA(0);


	for(i=0;i<100;i++);
	result[1]=_WSN_FPGA(1);

	/** _WSN_FPGA(1) = Humidity, ACC X **/


   /************ Temp: ******************/
   // the temperature value taken from the FPGA has	to be
   // substracted from 27315 in order to show Degree Celsius x 100
   
   t = t+1;
 
   if (t==t_T){
   			
   	Temp =  ( result[0] - (273.15*100) );	  // Degree Celsius x 100
   	t=0;
   }
   // c = 0;
   /*************************************/

   /************ Humidity ***************/
   // the humidity value taken form the FPGA has to be multipled
   // by 127.5 and divided by 100 in order to show H% x 100.
   h = h+1;
   if (h==t_H){
   	Hum =  ( (result[1]*127.0)/100 );
   	h=0;
   }
   // c = 0;
   /*************************************/
   
   /************ Light: ******************/
   l = l+1;
   if (l==t_L){
   	LDR=_WSN_ADC_conversion();
	l=0;
   }
   /**************************************/	


}
/*****************************************************************/

/****************** ZigBee read: *********************************/
/** ASCII  = Value of the character to wait.
/** getsmj = It allows to get caracters from the serial port and 
/** print them until ASCII arrives. 
**/

/*void _WSN_wait_answer(char ASCII,char getmsj)
{  
	charWait = ASCII;	
	flagWait = 1;	 
	while(flagWait);

/*	unsigned char serial_read,enable;

	enable = 1;
  
	     do
		{
			serial_read = _getkey(); 
	
			if (serial_read == ASCII) 
			{											 
			 	enable = 0;
			}
			else if (getmsj == 1)
			{
				putchar(serial_read);
			}			
		}while (enable != 0);
*/
//} */
/**************** ZigBee Configuration: ************************/
void _WSN_ZigBee_config()
{ 
  	_WSN_Write_UART("AT&F\r\0");
	flagWait=1;
	charWait='O';
	while(flagWait==1);
	_WSN_Write_UART("ATS00=0008\r\0");
	flagWait=1;
	charWait='O';
	while(flagWait==1);
	_WSN_Write_UART("ATS02=0008\r\0");
	flagWait=1;
	charWait='O';
	while(flagWait==1);
	_WSN_Write_UART("ATS03=0000000000000008\r\0");
	flagWait=1;
	charWait='O';
	while(flagWait==1);
	_WSN_Write_UART("AT+JN\r\0");
	flagWait=1;
	charWait='O';
	while(flagWait==1);

}
/******************* Message Detection: *************************
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
	flagWait = 0;
   //---- Peripheral Configurations: -------------
   _WS_Timer_Config(1);
   _WS_ADC_Config();
   _WSN_UART841_config(); 
   _WSN_ini_FPGA();
   _WSN_ZigBee_config();

   // --------------------------------------------

	   //printf ("Connected\n\r");

	   _WSN_Write_UART("Connected\n\r\0");

	   while (1)
	   {
	   			if(flagInterrupt == 1){
					_WSN_sensors_reading();
					/*if (t==0 || h==0 || l==0){
					sprintf(Mensaje,"AT+UCAST:0000=");
					}*/
					if (t==0)  {
							sprintf(aux,"AT+UCAST:0000=Temp:%d \r\0", Temp);
							_WSN_Write_UART(aux);
							flagWait=1;
							charWait='O';
							while(flagWait==1);
							/*_WSN_Write_UART("2\r\0");
							sprintf(aux,"Temp: %d ", Temp);
							strcat(Mensaje,aux);*/
					}
					if (h==0)  {
							sprintf(aux,"AT+UCAST:0000=Hum:%d \r\0", Hum);
							_WSN_Write_UART(aux);
							flagWait=1;
							charWait='O';
							while(flagWait==1);
							/*sprintf(aux,"Hum: %d ", Hum);
							strcat(Mensaje,aux);*/

					}
					if (l==0)  {
							sprintf(aux,"AT+UCAST:0000=LDR:%d \r\0", LDR);
							_WSN_Write_UART(aux);
							flagWait=1;
							charWait='O';
							while(flagWait==1);
							/*sprintf(aux,"LDR: %d ", LDR);
							strcat(Mensaje,aux);*/
					}
					/*if (t==0 || h==0 || l==0){
						sprintf(aux,"\r\0");
						strcat(Mensaje,aux);
						_WSN_Write_UART(Mensaje);
						flagWait=1;
						charWait='O';
						while(flagWait==1);
					} */
					flagInterrupt = 0;
				}	
				
				if(RX_flag == 1)
				{
					char *rx=RX_Buffer;
					while(*rx != '\0'){
						if(*rx=='T'){
						 rx++;
						 //t_T=(*rx) - '0';
						 //sprintf(aux,"recibido, t_T= %d, %d\n\r\0", t_T, *rx);
						 //_WSN_Write_UART(aux);
						 //_WSN_Write_UART("\nMensaje:");
						 //_WSN_Write_UART(RX_Buffer);
						}
						else if(*rx=='H'){
						 rx++;
						 //t_H=(*rx++)-'0';
						}
						else if(*rx=='L'){
						 rx++;
						 t_L=(*rx)-'0';
						 sprintf(aux,"recibido, t_L= %d, %d\n\r\0", t_L, *rx);
						 _WSN_Write_UART(aux);
						 _WSN_Write_UART("\nMensaje:");
						 _WSN_Write_UART(RX_Buffer);
						}
						else {
						_WSN_Write_UART("\nCaracter:");
						_WSN_Write_UART(rx);
						rx++;
						
						}
					}
					_WSN_Write_UART("recibido\n\r\0");
					RX_flag = 0;
				}		   		  
	   }

}
/****************************************************************/
