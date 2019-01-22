
#include <ADuC841.h>
#include <stdio.h>

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

unsigned char flagWait, RXFlag;
unsigned char charWait;
unsigned char tempo;

unsigned char RX_Buffer[20];
unsigned char flag, c;

unsigned int result,Accx_inicial, Accy_inicial, cuenta_acc;// LDR, Temp,Hum,cuenta_temp, cuenta_hum; 

unsigned int resulti[2];

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

	TI  = 1;   //bit1(SCON): Serial Port Transmit Interrupt Flag.
	ES  = 1;	// Serial Port interruption enable
	ET1 = 0;	// Timer 1 Interruption Disable	

	EA  = 1; 	// Global Enable Interruption Flag

	RXFlag = 0;
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
   
   
   flag = 1;

      
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
	   if(/*!flagWait && */SBUF == 't'){	/** If we are not waiting for a particular character, this condition can be removed, so that
	   					    every received byte will be stored in RX_Buffer	**/

		  _WSN_Read_UART(RX_Buffer);
		  //RX_Buffer;
		  // retransmiting the message just for testing:
		  //_WSN_Write_UART(RX_Buffer);
		  RXFlag = 1;
	   }
	   else if(flagWait == 1 && SBUF == charWait){ /** Condition for waiting an answer prompt, such as 'O' for "OK", etc. **/
		  flagWait = 0;
	   }
	 RI=0;
	 
	}//-------------------------------------

	ES = 1; // Esable Serial Interruption
}
/*****************************************************************/
/***************** Sensors reading functionalities: ***************/
void _WSN_sensors_reading(void){

   int result[2], i;
   static int cuenta = 0;
   cuenta++;

	
   	/** _WSN_FPGA(0) =  ACC Y **/
	
	if(cuenta%cuenta_acc == 0)
	{								
		resulti[1] = _WSN_FPGA(1); //accX 
		resulti[0] = _WSN_FPGA(0); //accY 
	
	}
	  
   	/** _WSN_FPGA(0) = Temperature, ACC Y **/
	/*
	if(cuenta%cuenta_temp == 0)
	{
		result[0] = _WSN_FPGA(0); 
		Temp = (result[0] - (273.15*100)); 
		_WSN_Write_UART("La temperatura son %.2f grados celsius\n", ((float)Temp)/100);
	}
		*/
	for(i=0;i<100;i++);

	


	/** _WSN_FPGA(1) = Humidity, ACC X **/ 
	/*
	if(cuenta%cuenta_hum == 0)
	{
		result[1] = _WSN_FPGA(1);  
    	Hum =  ( (result[1]*127.0)/100 );  
		_WSN_Write_UART("Hay una humedad del %.2f %c\n", ((float)Hum)/100,37);
	}
		*/

   /************ Temp: ******************/
   // the temperature value taken from the FPGA has	to be
   // substracted from 27315 in order to show Degree Celsius x 100				
   // Ej: Temp =  ( result - (273.15*100) );	  // Degree Celsius x 100
   // c = 0;
   /*************************************/
	

   /************ Humidity ***************/
   // the humidity value taken form the FPGA has to be multipled
   // by 127.5 and divided by 100 in order to show H% x 100.
   // Ej: Hum =  ( (result*127.0)/100 );
   // c = 0;
   /*************************************/


   /************ Light: ******************/

   /**************************************/	


}
/*****************************************************************/

/****************** ZigBee read: *********************************/
/** ASCII  = Value of the character to wait.
/** getsmj = It allows to get caracters from the serial port and 
/** print them until ASCII arrives. 
**/

void _WSN_wait_answer(char ASCII,char getmsj)
{ 
	unsigned char serial_read,enable;
	charWait = ASCII;	
	flagWait = 1;	 
	while(flagWait);  
	

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
}
/****************** ZigBee read: *********************************/
/** ASCII  = Value of the character to wait.
/** getsmj = It allows to get caracters from the serial port and 
/** print them until ASCII arrives. 
**/


/**************** ZigBee Configuration: ************************/
void _WSN_ZigBee_config(void)
{ 		
	char temp_read;
  	_WSN_Write_UART ("AT&F\r\0");
  	_WSN_wait_answer('K',0);
  	_WSN_Write_UART ("ATS12=0590\r\0");
 	_WSN_wait_answer('K',0);		
	_WSN_Write_UART("ATS00=1000\r\0");
	_WSN_wait_answer('K',0);
	_WSN_Write_UART("ATS02=0100\r\0");
	_WSN_wait_answer('K',0);
	_WSN_Write_UART("ATS03=1111111111111112\r\0");
	_WSN_wait_answer('K',0);
	_WSN_Write_UART("AT+JN\r\0");
	_WSN_wait_answer('K',0);
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
/**************** ZigBee Configuration: ************************/
/*void _WSN_ZigBee_config(char type)
{ 

	char temp_read;
	_WSN_Write_UART("AT&F\r");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATS00=1000\r");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATS02=0100\r");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATS03=1111111111111112\r");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("AT+JN\r");
	_WSN_wait_answer('O',0);
	/*scanf("%c",&temp_read);
	if(temp_read == 'O')
	{
		_WSN_Write_UART("ATS02=0100\r");
		scanf("%c",&temp_read);	
			if(temp_read == 'O')
			{
				_WSN_Write_UART("ATS03=1111111111111112\r");
				scanf("%c",&temp_read);
				if(temp_read == 'O')
				{	  
					_WSN_Write_UART("AT+JN\r");
					scanf("%c",&temp_read);
					if(temp_read == 'O')
					{	
						//OK
					}
				}
			}
	}
}	
*/
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

   c = 'O';
   flag = 0;
   _WS_Timer_Config(1);
   _WS_ADC_Config();
   _WSN_UART841_config();
   _WSN_ini_FPGA();	  
   _WSN_ZigBee_config();
  
   //entrada cuenta humedad y temperatura por defecto
   //cuenta_temp = 1;
   //cuenta_hum = 5;
   
   cuenta_acc = 5;
   // --------------------------------------------
	_WSN_Write_UART("Se va a guardar la posicion incial de la tarjeta \n\0");
							  			
	//Accx_inicial = _WSN_FPGA(1); //accX 
	//Accy_inicial = _WSN_FPGA(0); //accY 

	   _WSN_Write_UART ("Connected\n\r");	   			   

	   while (1)
	   {
	   	   if (flag == 1){
			 _WSN_Write_UART("Empezamos a leer\r\0");
			 _WSN_Write_UART("AT+UCAST:0000,\0");
			 _WSN_Write_UART("(X,Y): 15,26\n\0");
			//_WSN_sensors_reading();
		_WSN_Write_UART("AT+UCAST:0000,(X,Y): %d, %d\n",(resulti[1]-Accx_inicial), (resulti[0]-Accy_inicial));
		_WSN_Write_UART("(X,Y): %d, %d\n",(resulti[1]-Accx_inicial), (resulti[0]-Accy_inicial));	
		   
			/********* SHT11 Sensor Layer *************************/
		 
		    /*******************************************************/
			
			/********* ACC Sensor layer **************************

			/*****************************************************/			
			if(RXFlag == 1){
			  //val1 = RX_Buffer[1];

			}
			flag = 0;

		}				
	   	 	   		  
	   }

}
/****************************************************************/
