
#include <ADuC841.h>
#include <stdio.h>
#include <windows.h>


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

unsigned int result,Temp,Hum, LDR; 
unsigned int estado=3, est_com=1; 
//unsigned char configvalue=1;
unsigned int resulti[2];
								   

/***************** Timer Configuration: **************************/
void _WS_Timer_Config(char value)//this function follows the frequency of the checking of the sensors
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
void _WS_ADC_Config (void)	   //it'll help us get information from the LDR
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
	ES  = 0;	// Serial Port interruption disable
	ET1 = 0;	// Timer 1 Interruption Disable	

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
int _WSN_FPGA(bit sensorSelector)	 //if we set this function at 0, we get 0 for the temp and one for the humidity value, which is a percentage
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
void _WSN_interrupt_TimeInterval() interrupt 10 using 3    //this is the interruption of the timer. we jump into this function and we process a flag we're activating
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

/*****************************************************************/

/***************** Sensors reading functionalities: ***************/
void _WSN_sensors_reading(void){				 //we gotta read the humidity and the temperature 

   int result[2], i;

   	//_WSN_FPGA(0) = Temperature, ACC Y;
	Temp=_WSN_FPGA(0);
	Temp =  ( Temp - (273.15*100) );
	
	for(i=0;i<1000;i++);

	//_WSN_FPGA(1) = Humidity, ACC X;
	Hum=_WSN_FPGA(1);
	Hum =  ( (Hum*127.0)/100 );

   /************ Temp: ******************/
   //resulti[0]=result[0]*100/27315;
   // the temperature value taken from the FPGA has	to be
   // substracted from 27315 in order to show Degree Celsius x 100				
   // Ej: Temp =  ( result - (273.15*100) );	  // Degree Celsius x 100
   // c = 0;
   /*************************************/

   /************ Humidity ***************/
   //resulti[1] = result[1]*127.5/100;
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
/**************** ZigBee Configuration: ************************/
/*void _WSN_ZigBee_config(char type)
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
void maquinaEstados()
{
	// se inicializa como libre. estado 3 = libre, estado 2 = ocupado, estado 1 = recién ocupado

	if (estado == 3)
	{
		if(LDR<1500)
		{
			estado=2;
		}
	}
	
	else if (estado == 2)
	{
		if(LDR>=1500)
		{
			estado=3;		//El coche llega y se detecta aumento en temperatura por el motor.
		}
	}
}

void imprimirestado()
{
 	if(estado==1)
	{
	 	printf("Nodo 1: La plaza acaba de ocuparse\n");
	}
	if(estado==2)
	{
	 	printf("Nodo 1: La plaza estaba ocupada\n");
	}
	if(estado==3)
	{
	 	printf("Nodo 1: La plaza esta libre\n");
	}
}

void MEST_Comunicaciones()
{
	//est_com se inicializa a 1, 1 es el modo de enviar la información cada x tiempo y 2 se envía la información cuando se le pida	
}


void main()
{
  
   //---- Peripheral Configurations: -------------
	_WSN_ini_FPGA();
	_WS_ADC_Config();
	_WSN_UART841_config();
	

	//_WSN_ZigBee_config();
	
   c = 'O';
   flag = 0;

   // --------------------------------------------
 
	  printf("Connected\n\r");
	  _WS_Timer_Config(1);	   			   				
		while (1)
	   {
	   	 	   	   
	   	   if (flag == 1)
		   {
			//_WSN_sensors_reading();
			 
			/********* SHT11 Sensor Layer *************************/
			//printf("Temperatura= %d\n",Temp);
			//printf("Humedad= %d\n",Hum);

			LDR=_WSN_ADC_conversion();
			printf("LDR= %d\n",LDR);
			
		   /*******************************************************/
			
			/********* ACC Sensor layer **************************

			/*****************************************************/			

			flag = 0;
			maquinaEstados();
			imprimirestado();
			
		  	}
			
		}
}
	
/****************************************************************/
