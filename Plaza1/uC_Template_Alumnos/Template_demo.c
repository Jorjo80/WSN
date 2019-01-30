
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
int f=0;
int q=0;
int j =0; 
unsigned char DATA_L;
unsigned char DATA_H;
unsigned int datain;

unsigned char flagWait, RX_flag=0;
unsigned char charWait;
unsigned char flagInterrupt=0;

unsigned char flag, c;

unsigned int result,Temp,Hum, LDR; 
unsigned int estado=3, est_com=1; 

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
	ES  = 1;	// Serial Port interruption enable
	ET1 = 0;	// Timer 1 Interruption Disable	

	EA  = 1; 	// Global Enable Interruption Flag

	RX_flag = 0;
}

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


	if (RI == 1) {
	   
	   if(/*!flagWait && */SBUF == 'w'|| SBUF == 'W'){	// If we are not waiting for a particular character, this condition can be removed, so that every received byte will be stored in RX_Buffer	
		   RX_flag = 1;
	   }
	   	else if(/*!flagWait && */SBUF == 'z'|| SBUF =='Z'){	// If we are not waiting for a particular character, this condition can be removed, so that every received byte will be stored in RX_Buffer	
		  
		  RX_flag = 2;
	   }

	   	else if(/*!flagWait && */SBUF == 't'|| SBUF =='T'){	// If we are not waiting for a particular character, this condition can be removed, so that every received byte will be stored in RX_Buffer	
	  
		  RX_flag = 3;
	   }
	   	else if(/*!flagWait && */SBUF == 'f'|| SBUF =='F')
		{
			RX_flag=4;	// If we are not waiting for a particular character, this condition can be removed, so that every received byte will be stored in RX_Buffer	
	 	
	   	}

	   else if(flagWait == 1 && SBUF == charWait){ // Condition for waiting an answer prompt, such as 'O' for "OK", etc. 
		  flagWait = 0;
	   }
	 RI=0;
	}//-------------------------------------

	ES = 1; // Esable Serial Interruption
}

																	
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

	Temp=_WSN_FPGA(0);
	Temp =  ( Temp - (273.15*100) );
	
	for(i=0;i<1000;i++);


	Hum=_WSN_FPGA(1);
	Hum =  ( (Hum*127.0)/100 );

}

void _WSN_wait_answer(char ASCII,char getmsj)
{  
	charWait = ASCII;	
	flagWait = 1;	 
	while(flagWait);
	flagWait = 1;
}
/**************** ZigBee Configuration: ************************/
void _WSN_ZigBee_config(void)
{ 		
	
	_WSN_Write_UART("ATS00=0040\n\r\0");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATS02=0007\n\r\0");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATS03=1111111111111117\n\r\0");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("ATZ\n\r\0");
	_WSN_wait_answer('O',0);
	_WSN_Write_UART("AT+JN\n\r\0");
	_WSN_wait_answer('O',0);
}
/******************* Message Detection: *************************/

/******************* Main Function: *****************************/
void imprimirestado()
{
 	if(estado==2)
	{
		_WSN_Write_UART("AT+UCAST:0000=\0");
	 	_WSN_Write_UART("Plaza 1: La plaza esta ocupada\n\r\0");
	}
	if(estado==3)
	{	
		_WSN_Write_UART("AT+UCAST:0000=\0");
	 	_WSN_Write_UART("Plaza 1: La plaza esta libre\n\r\0");
	}
	RX_flag=0;
}

void maquinaEstados()
{
	// se inicializa como libre. estado 3 = libre, estado 2 = ocupado, estado 1 = recién ocupado

	if (estado == 3)
	{
		if(LDR<1500)
		{
			estado=2;
			RX_flag = 6;
			f=1;
		}
	}
	
	else if (estado == 2)
	{
		if(LDR>=1500)
		{
			estado=3;
			RX_flag = 6;		//El coche llega y se detecta aumento en temperatura por el motor.
			f=2;
		}
	}


}


void EnviarDatos(void)
{ 	
	if(RX_flag==1)
	{			 
		est_com=1;
		RX_flag=0;
		j=0;
	}
	if(RX_flag==2)
	{
		est_com=2;
		RX_flag=0;
		j=0;
		
	}
	
	if(est_com==1)
	{
	  if (flag == 1)
		{
			//_WSN_sensors_reading();
			 
			/********* SHT11 Sensor Layer *************************/

			if(q<f)q++;
			else
			{
		   	
			LDR=_WSN_ADC_conversion();
			maquinaEstados();
			imprimirestado(); 	
			q=0;
			}
			flag = 0;			
		  }  
	}
	if (est_com==2)
	{  		
	   	LDR=_WSN_ADC_conversion();
		maquinaEstados();
		flag = 0;
		if(RX_flag==3)
		{	 
			imprimirestado();
		}
		if(RX_flag==6)
		{	 
			imprimirestado();
			
		
		}
		else if(RX_flag==4)
		{			 
	  	 	if(estado==3)
		 	{
		 		_WSN_Write_UART("AT+UCAST:0000=\0");
				_WSN_Write_UART("Plaza 1 está libre\n\r\0");
		 	}
			
		}
		else if(RX_flag==5)
		{	
			if(estado==2)
		 	{
		 		_WSN_Write_UART("AT+UCAST:0000=\0");
				_WSN_Write_UART("Plaza 1 está ocupada\n\r\0");
		 	}
			
		}
		else
		{
			while(j<1)
			{	
				
				_WSN_Write_UART("AT+UCAST:0000=\0");
				_WSN_Write_UART("Esperando llamada\n\r\0");
				j++;
			}
		}
		RX_flag=0;
	}
			
}



void main()
{	 

	 _WSN_UART841_config();
   //---- Peripheral Configurations: -------------
	_WSN_ini_FPGA();
	_WS_ADC_Config();
	_WSN_UART841_config();
	_WSN_ZigBee_config();
	
   c = 'O';
   flag = 0;

   // --------------------------------------------
 
	  	_WSN_Write_UART("Connected\n\r\0");
	  	_WS_Timer_Config(1);	   			   				
		while (1)
		{	 	   	  
			EnviarDatos();		
		}
}
	
/****************************************************************/
