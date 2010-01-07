/*
 * Copyright (c) 2009 Justin F. Knotzke (jknotzke@shampoo.ca)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*********************************************************************************
 * Logomatic V2 Firmware
 * Sparkfun Electronics 2008
 * ******************************************************************************/

/*******************************************************
 * 		     Header Files
 ******************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "LPC21xx.h"

//UART0 Debugging
#include "serial.h"
#include "rprintf.h"


//Needed for main function calls
#include "main_msc.h"
#include "fat16.h"
#include "armVIC.h"
#include "itoa.h"
#include "rootdir.h"
#include "sd_raw.h"


#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <string.h>

#include "serial.h"
#include "libant.h"

/*******************************************************
 * 		     Global Variables
 ******************************************************/

#define ON	1
#define OFF	0

char RX_array1[512];
char RX_array2[512];
char log_array1 = 0;
char log_array2 = 0;
short RX_in = 0;
char get_frame = 0;

signed int stringSize;
struct fat16_file_struct * handle; //Actual Log File.
char stringBuf[256];


// Default Settings
static int baud = 9600;
static char trig = '$';
static short frame = 100;

//Timestamp
struct timestamp
{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
};


/*******************************************************
 * 		 Function Declarations
 ******************************************************/

void Initialize(void);

void setup_uart0(int newbaud, char want_ints);

void mode_0(void);
void mode_1(void);
void mode_2(void);
void mode_action(void);

void Log_init(void);
void test(void);
void statLight(int num, int onoff);
void AD_conversion(int regbank);

void feed(void);

static void UART0ISR(void); //__attribute__ ((interrupt("IRQ")));

void FIQ_Routine(void) __attribute__ ((interrupt("FIQ")));
void SWI_Routine(void) __attribute__ ((interrupt("SWI")));
void UNDEF_Routine(void) __attribute__ ((interrupt("UNDEF")));

void fat_initialize(void);

void delay_ms(int count);
void flashBoobies(int num_of_times);

void ANTAP1_Config(void);
void ANTAP1_Reset(void);
void ANTAP1_AssignCh(void);
void ANTAP1_SetChId(void);
void ANTAP1_SetChRFFreq(void);
void ANTAP1_SetChPeriod(void);
void ANTAP1_OpenCh(void);
void ANTAP1_AssignNetwork(void);
void ANTAP1_SetSearchTimeout(void);
void ANTAP1_RequestChanID(void);
void set_time(void);
struct timestamp get_time(void);


#define chanNum   0x00        // ChanNum
#define chanType  0x00 // ChanType
#define netNum    0x00    // NetNum
#define devNum 0x00
#define rate  0x1f86 //HRM

void ANTAP1_Config (void)
{
    set_time();
    ANTAP1_Reset();
    delay_ms(50);
    ANTAP1_AssignCh();
    delay_ms(50);
    ANTAP1_SetChId();
    delay_ms(50);
    ANTAP1_AssignNetwork();
    delay_ms(50);
    ANTAP1_SetSearchTimeout();
    delay_ms(50);
    ANTAP1_SetChRFFreq();
    delay_ms(50);
    ANTAP1_SetChPeriod();
    delay_ms(50);
    //ANTAP1_RequestChanID();
    //delay_ms(50);
    ANTAP1_OpenCh();
    delay_ms(50);

    //get_time();
}


void ANTAP1_SetSearchTimeout(void)
{
    unsigned char i;
    unsigned char setup[6];

    setup[0] = 0xa4; // SYNC Byte
    setup[1] = 0x02; // LENGTH Byte
    setup[2] = 0x44; // ID Byte
    setup[3] = 0x00; // Data Byte N (N=LENGTH)
    setup[4] = 0x1e; // Checksum
    setup[5] = (0xa4^0x02^0x44^0x00^0x1e);
    
    for(i = 0 ; i < 6 ; i++)
       putc_serial0(setup[i]);
   
}


// Resets module
void ANTAP1_Reset (void) 
{
    unsigned char i;
    unsigned char setup[5];
    
    setup[0] = 0xa4; // SYNC Byte
    setup[1] = 0x01; // LENGTH Byte
    setup[2] = 0x4a; // ID Byte
    setup[3] = 0x00; // Data Byte N (N=LENGTH)
    setup[4] = 0xef; // Checksum
    
    for(i = 0 ; i < 5 ; i++)
       putc_serial0(setup[i]);
}

void ANTAP1_AssignNetwork(void)
{
    unsigned char i;
    unsigned char setup[13];

    setup[0] = 0xa4; //Sync
    setup[1] = 0x09; //Length
    setup[2] = MESG_NETWORK_KEY_ID;
    setup[3] = 0x00; // network number 
    setup[4] = 0xb9;
    setup[5] = 0xa5;     
    setup[6] = 0x21;    
    setup[7] = 0xfb; 
    setup[8] = 0xbd; 
    setup[9] = 0x72; 
    setup[10] = 0xc3; 
    setup[11] = 0x45; 
    setup[12] = (0xa4^0x09^MESG_NETWORK_KEY_ID^0x00^0xb9^0xa5^0x21^0xfb^0xbd^0x72^0xc3^0x45);
    
    for(i = 0 ; i < 13 ; i++)
      putc_serial0(setup[i]);
}


// Assigns CH=0, CH Type=00(RX), Net#=0
void ANTAP1_AssignCh (void) 
{
    unsigned char i;
    unsigned char setup[7];
   
    setup[0] = 0xa4;
    setup[1] = 0x03;
    setup[2] = 0x42;
    setup[3] = chanNum;        // ChanNum
    setup[4] = chanType;    // ChanType
    setup[5] = netNum;        // NetNum
    setup[6] = (0xa4^0x03^0x42^chanNum^chanType^netNum);
    
    for(i = 0 ; i < 7 ; i++)
      putc_serial0(setup[i]);
}

// Assigns CH=0, RF Freq
void ANTAP1_SetChRFFreq (void) 
{
    unsigned char i;
    unsigned char setup[6];
   
    setup[0] = 0xa4;
    setup[1] = 0x02;
    setup[2] = 0x45;
    setup[3] = 0x00;        // ChanNum
    setup[4] = 0x39;        // RF Freq
    setup[5] = (0xa4^0x02^0x45^0x00^0x39);
    
    for(i = 0 ; i < 6 ; i++)
        putc_serial0(setup[i]);

}

// Assigns CH=0, RF Freq
void ANTAP1_RequestChanID(void) 
{
    unsigned char i;
    unsigned char setup[6];
    
    setup[0] = 0xa4;
    setup[1] = 0x02;
    setup[2] = 0x4d;
    setup[3] = 0x00;        // ChanNum
    setup[4] = 0x54;        //Extra Info
    setup[5] = (0xa4^0x02^0x4d^0x00^0x54);
    
    for(i = 0 ; i < 6 ; i++)
        putc_serial0(setup[i]);
}

void ANTAP1_SetChPeriod (void) 
{
    unsigned char i;
    unsigned char setup[7];
    
    setup[0] = 0xa4;
    setup[1] = 0x03;
    setup[2] = 0x43;
    setup[3] = 0x00;
    setup[4] = 0x86;
    setup[5] = 0x1f;
    setup[6] = (0xa4^0x03^0x43^0x00^0x86^0x1f);
    
    for(i = 0 ; i < 7 ; i++)
         putc_serial0(setup[i]);
  
}

// Assigns Device#=0000 (wildcard), Device Type ID=00 (wildcard), Trans Type=00 (wildcard)
void ANTAP1_SetChId (void) 
{
    unsigned char i;
    unsigned char setup[9];
    
    setup[0] = 0xa4;
    setup[1] = 0x05;
    setup[2] = 0x51;
    setup[3] = 0x00;
    setup[4] = 0x00;
    setup[5] = 0x00;
    setup[6] = 0x00;
    setup[7] = 0x00;
    setup[8] = (0xa4^0x05^0x51^0x00^0x00^0x00^0x00^0x00);
    
    for(i = 0 ; i < 9 ; i++)
       putc_serial0(setup[i]);
}

// Opens CH 0
void ANTAP1_OpenCh (void) 
{
    unsigned char i;
    unsigned char setup[5];
    
    setup[0] = 0xa4;
    setup[1] = 0x01;
    setup[2] = 0x4b;
    setup[3] = 0x00;
    setup[4] = (0xa4^0x01^0x4b^0x00);
    
    for(i = 0 ; i < 5 ; i++)
      putc_serial0(setup[i]);

}



/*******************************************************
 * 		     	MAIN
 ******************************************************/

int main (void)
{
	int i;
	char name[32];
	int count = 0;

	enableFIQ();

	Initialize();

	fat_initialize();		

	setup_uart0(4800, 0);

	// Flash Status Lights
	for(i = 0; i < 5; i++)
	{
		statLight(0,ON);
		delay_ms(50);
		statLight(0,OFF);
		statLight(1,ON);
		delay_ms(50);
		statLight(1,OFF);
	}

	//Log_init();

	count++;
	string_printf(name,"ANT%02d.gce",count);
	while(root_file_exists(name))
	{
		count++;
		if(count == 250) 
		{
			rprintf("Too Many Logs!\n\r");
			while(1)
			{
				statLight(0,ON);
				statLight(1,ON);
				delay_ms(1000);
				statLight(0,OFF);
				statLight(1,OFF);
				delay_ms(1000);
			}

		}
		string_printf(name,"ANT%02d.gce",count);
	}


 	handle = root_open_new(name);

	sd_raw_sync();	

	mode_0();

	return 0;
}

//Set the time
struct timestamp get_time(void)
{

       struct timestamp ts;
       //Test to see if you can create a timestamp.
       PCONP |= (1<<9); //make sure RTC is powered, just to be sure 
       CCR = 0x12; //disable RTC, set xtal flag true 

       //update time registers 
       ts.year = YEAR; 
       ts.month = MONTH;
       ts.day = DOM;
       ts.hour = HOUR;
       ts.minute = MIN;
       ts.second = SEC;
       
       CCR = 0x11; //enable RTC, keep xtal flag true 

       return ts;

}


//Set the time
void set_time(void)
{
       //Test to see if you can create a timestamp.
       PCONP |= (1<<9); //make sure RTC is powered, just to be sure 
       CCR = 0x12; //disable RTC, set xtal flag true 

       //update time registers 
       YEAR = 2009; 
       MONTH = 12;
       DOM = 31;
       HOUR = 12;
       MIN = 26;
       SEC = 0;
       
       CCR = 0x11; //enable RTC, keep xtal flag true 

}



/*******************************************************
 * 		     Initialize
 ******************************************************/

#define PLOCK 0x400

void Initialize(void)
{
	//rprintf_devopen(putc_serial0);

    init_serial0(4800);
	PINSEL0 = 0xCF351505;
	PINSEL1 = 0x15441801;
	IODIR0 |= 0x00000884;
	IOSET0 = 0x00000080;

	S0SPCR = 0x08;  // SPI clk to be pclk/8
	S0SPCR = 0x30;  // master, msb, first clk edge, active high, no ints

}

void feed(void)
{
	PLLFEED=0xAA;
	PLLFEED=0x55;
}

static void UART0ISR(void)
{
	char temp;

 	if(RX_in < 512)
	{
		RX_array1[RX_in] = U0RBR;

		RX_in++;

		if(RX_in == 512) log_array1 = 1;
	}
	else if(RX_in >= 512)
	{
		RX_array2[RX_in-512] = U0RBR;
		RX_in++;

		if(RX_in == 1024)
		{
			log_array2 = 1;
			RX_in = 0;
		}
	}


	temp = U0IIR; // Have to read this to clear the interrupt 

	VICVectAddr = 0;

}

static void UART0ISR_2(void)
{
	char temp;
	temp = U0RBR;

	if(temp == trig){ get_frame = 1; }

	if(get_frame)
	{
		if(RX_in < frame)
		{
			RX_array1[RX_in] = temp;
			RX_in++;

			if(RX_in == frame)
			{
				RX_array1[RX_in] = 10; // delimiters
				RX_array1[RX_in + 1] = 13;
				log_array1 = 1;
				get_frame = 0;
			}
		}
		else if(RX_in >= frame)
		{
			RX_array2[RX_in - frame] = temp;
			RX_in++;

			if(RX_in == 2*frame)
			{
				RX_array2[RX_in - frame] = 10; // delimiters
				RX_array2[RX_in + 1 - frame] = 13;
				log_array2 = 1;
				get_frame = 0;
				RX_in = 0;
			}
		}
	}

	temp = U0IIR; // have to read this to clear the interrupt

	VICVectAddr = 0;
}

void FIQ_Routine(void)
{
	char a;
	int j;

	statLight(0,ON);
	for(j = 0; j < 5000000; j++);
	statLight(0,OFF);
	a = U0RBR;

	a = U0IIR;  // have to read this to clear the interrupt
}

void SWI_Routine(void)
{
	while(1);
}

void UNDEF_Routine(void)
{
	statLight(0,ON);
}

void setup_uart0(int newbaud, char want_ints)
{
	baud = newbaud;
	U0LCR = 0x83;   // 8 bits, no parity, 1 stop bit, DLAB = 1

	if(baud == 1200)
	{
		U0DLM = 0x0C;
		U0DLL = 0x00;
	}
	else if(baud == 2400)
	{
		U0DLM = 0x06;
		U0DLL = 0x00;
	}
	else if(baud == 4800)
	{
		U0DLM = 0x03;
		U0DLL = 0x00;
	}
	else if(baud == 9600)
	{
		U0DLM = 0x01;
		U0DLL = 0x80;
	}
	else if(baud == 19200)
	{
		U0DLM = 0x00;
		U0DLL = 0xC0;
	}
	else if(baud == 38400)
	{
		U0DLM = 0x00;
		U0DLL = 0x60;
	}
	else if(baud == 57600)
	{
		U0DLM = 0x00;
		U0DLL = 0x40;
	}
	else if(baud == 115200)
	{
		U0DLM = 0x00;
		U0DLL = 0x20;
	}

	U0FCR = 0x01;
	U0LCR = 0x03;   

	if(want_ints == 1)
	{
		enableIRQ();
		VICIntSelect &= ~0x00000040;
		VICIntEnable |= 0x00000040;
		VICVectCntl1 = 0x26;
		VICVectAddr1 = (unsigned int)UART0ISR;
		U0IER = 0x01;
	}
	else if(want_ints == 2)
	{
		enableIRQ();
		VICIntSelect &= ~0x00000040;
		VICIntEnable |= 0x00000040;
		VICVectCntl2 = 0x26;
		VICVectAddr2 = (unsigned int)UART0ISR_2;
		U0IER = 0X01;
	}
	else if(want_ints == 0)
	{
		VICIntEnClr = 0x00000040;
		U0IER = 0x00;
	}
}
void statLight(int statLightnum, int onoff)
{
	if(statLightnum) // Stat 1
	{
		if(onoff){ IOCLR0 = 0x00000800; } // On
		else { IOSET0 = 0x00000800; } // Off
	}
	else // Stat 0 
	{
		if(onoff){ IOCLR0 = 0x00000004; } // On
		else { IOSET0 = 0x00000004; } // Off
	}
}


void mode_0(void) // Auto UART mode
{
	setup_uart0(baud,1);
	stringSize = 512;
	ANTAP1_Config();
    mode_action();
}


void mode_action(void)
{
	int j;
	while(1)
	{

		if(log_array1 == 1)
		{
			statLight(0,ON);
			if(fat16_write_file(handle,(unsigned char *)RX_array1, stringSize) < 0)
			{
				while(1)
				{
					statLight(0,ON);
					for(j = 0; j < 500000; j++)
						statLight(0,OFF);
					statLight(1,ON);
					for(j = 0; j < 500000; j++)
						statLight(1,OFF);
				}
			}

			sd_raw_sync();
			statLight(0,OFF);
			log_array1 = 0;
		}

		if(log_array2 == 1)
		{
			statLight(1,ON);
			
            if(fat16_write_file(handle,(unsigned char *)RX_array2, stringSize) < 0)
			{
				while(1)
				{
					statLight(0,ON);
					for(j = 0; j < 500000; j++)
						statLight(0,OFF);
					statLight(1,ON);
					for(j = 0; j < 500000; j++)
						statLight(1,OFF);
				}
			}

			sd_raw_sync();
			statLight(1,OFF);
			log_array2 = 0;
		}

		if((IOPIN0 & 0x00000008) == 0) // if button pushed, log file & quit
		{
			VICIntEnClr = 0xFFFFFFFF;

			if(RX_in < 512)
			{
				fat16_write_file(handle, (unsigned char *)RX_array1, RX_in);
				sd_raw_sync();
			}
			else if(RX_in >= 512)
			{
				fat16_write_file(handle, (unsigned char *)RX_array2, RX_in - 512);
				sd_raw_sync();
			}
			while(1)
			{
				statLight(0,ON);
				for(j = 0; j < 500000; j++);
				statLight(0,OFF);
				statLight(1,ON);
				for(j = 0; j < 500000; j++);
				statLight(1,OFF);
			}
		}
	}

}


void AD_conversion(int regbank)
{
	int temp = 0, temp2;

	if(!regbank) // bank 0
	{
		AD0CR |= 0x01000000; // start conversion
		while((temp & 0x80000000) == 0)
		{
			temp = AD0DR;
		}
		temp &= 0x0000FFC0;
		temp2 = temp / 0x00000040;

		AD0CR = 0x00000000;
	}
	else	    // bank 1
	{
		AD1CR |= 0x01000000; // start conversion
		while((temp & 0x80000000) == 0)
		{
			temp = AD1DR;
		}
		temp &= 0x0000FFC0;
		temp2 = temp / 0x00000040;

		AD1CR = 0x00000000;
	}

	//rprintf("%d", temp2);
	//rprintf("   ");

}

void fat_initialize(void)
{
	if(!sd_raw_init())
	{
		//rprintf("SD Init Error\n\r");
		while(1);
	}

	if(openroot())
	{ 
		//rprintf("SD OpenRoot Error\n\r");
	}
}

/*
void write_debug(char *debug)
{
	stringSize = strlen(debug);
	fat16_write_file(dbgfd, (unsigned char*)debug, stringSize);
	sd_raw_sync();
}
*/

void flashBoobies(int num_of_times)
{
	// Flash Status Lights
	for(int i = 0; i < num_of_times; i++)
	{
		statLight(0,ON);
		statLight(1,ON);
		delay_ms(100);
		statLight(0,OFF);
		statLight(1,OFF);
		delay_ms(100); 
	}

}

void delay_ms(int count)
{
	int i;
	count *= 10000;
	for(i = 0; i < count; i++)
		asm volatile ("nop");
}
