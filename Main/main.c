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
 *                   Header Files
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

#include "autoant.h"

/*******************************************************
 *                   Global Variables
 ******************************************************/

#define ON      1
#define OFF     0

#define chanType  0x00 // ChanType
#define netNum    0x00    // NetNum
#define rate  0x1f86 //HRM
#define MESG_NETWORK_KEY_ID	 0x46
#define MESG_TX_SYNC 0xa4
#define MESG_BROADCAST_DATA_ID 0x4E

#define DEVTYPE_HRM	0x78	/* ANT+ HRM */
#define DEVTYPE_BIKE	0x79	/* ANT+ Bike speed and cadence */
//#define DEVTYPE_FOOT	0x7c	/* ANT+ Foot pod */
#define DEVTYPE_PWR	0x0b	/* ANT+ Power meter */

#define DEVPERIOD_HRM	0x86	/* ANT+ HRM */
#define DEVPERIOD_BIKE	0x96	/* ANT+ Bike speed and cadence */
//#define DEVPERIOD_FOOT	0xc6	/* ANT+ Foot pod */
#define DEVPERIOD_PWR	0xf6	/* ANT+ Power meter */

#define FALSE 0
#define TRUE 1


char RX_array1[512];
char RX_array2[512];
char log_array1 = 0;
char log_array2 = 0;
short RX_in = 0;
char get_frame = 0;

signed int stringSize;
struct fat16_file_struct * handle; //Actual Log File.
struct fat16_file_struct * out_handle;

char stringBuf[256];
struct timestamp ts;

int inMsg = FALSE;
int msgN = 0;
int size = 0;

int seen[4];
int isBroadCast = FALSE;
int currentChannel=-1;

// Default Settings
<<<<<<< HEAD
static int baud = 115200; //38400; //9600;
=======
static int baud = 9600; //115200; //38400; //9600;
>>>>>>> d8e33a4
static char trig = '$';
static short frame = 100;

//Timestamp
struct timestamp
{
    char hour;
    char minute;
    char second;
};


/*******************************************************
 *               Function Declarations
 ******************************************************/

void Initialize(void);

void setup_uart0(int newbaud, char want_ints);

void mode_0(void);
void mode_action(void);

void statLight(int num, int onoff);
void AD_conversion(int regbank);

void feed(void);

static void UART0ISR(void); //__attribute__ ((interrupt("IRQ")));

void FIQ_Routine(void) __attribute__ ((interrupt("FIQ")));
void SWI_Routine(void) __attribute__ ((interrupt("SWI")));
void UNDEF_Routine(void) __attribute__ ((interrupt("UNDEF")));

void fat_initialize(void);

void delay_ms(int count);
/*
void ANTAP1_Config(void);
void ANTAP1_Reset(void);
void ANTAP1_AssignCh(unsigned char);
void ANTAP1_SetChId(unsigned char, unsigned char deviceType);
void ANTAP1_SetChRFFreq(unsigned char);
void ANTAP1_SetChPeriod(unsigned char, unsigned char);
void ANTAP1_OpenCh(unsigned char);
void ANTAP1_AssignNetwork(unsigned char);
void ANTAP1_SetSearchTimeout(unsigned char);
*/
void ANTAP1_RequestChanID(unsigned char);
void set_time(void);
void get_time(void);
int parseANT(unsigned char chr);
void add_time_stamp(void);
void must_we_write(void);

static const char antap1_config[] =
<<<<<<< HEAD
=======
    "# baudrate = 115200\n"
>>>>>>> d8e33a4
    "w[4a][00] # reset\n"
    "d500\n"
    "# Assign the network only once, since it's always the same\n"
    "w[46][01][b9][a5][21][fb][bd][72][c3][45] # ANTAP1_AssignNetwork(0x01)\n"
    "\n"
    "# HR\n"
<<<<<<< HEAD
    "w[24][00][00][01] # ANTAP1_AssignCh(0x00)\n"
=======
    "w[42][00][00][01] # ANTAP1_AssignCh(0x00)\n"
>>>>>>> d8e33a4
    "d50\n"
    "w[51][00][00][00][78][00] # ANTAP1_SetChId(0x00,DEVTYPE_HRM)\n"
    "d50\n"
    "w[44][00][1e] # ANTAP1_SetSearchTimeout(0x00)\n"
    "d50\n"
    "w[45][00][39] # ANTAP1_SetChRFFreq(0x00)\n"
    "d50\n"
    "w[43][00][86][1f] # ANTAP1_SetChPeriod(0x00, DEVPERIOD_HRM)\n"
    "d50\n"
    "w[4b][00] # ANTAP1_OpenCh(0x00)\n"
    "d50\n"
    "\n"
    "# Power\n"
<<<<<<< HEAD
    "w[24][01][00][01] # ANTAP1_AssignCh(0x01)\n"
=======
    "w[42][01][00][01] # ANTAP1_AssignCh(0x01)\n"
>>>>>>> d8e33a4
    "d50\n"
    "w[51][01][00][00][0b][00] # ANTAP1_SetChId(0x01,DEVTYPE_PWR)\n"
    "d50\n"
    "w[44][01][1e] # ANTAP1_SetSearchTimeout(0x01)\n"
    "d50\n"
    "w[45][01][39] # ANTAP1_SetChRFFreq(0x01)\n"
    "d50\n"
    "w[43][01][f6][1f] # ANTAP1_SetChPeriod(0x01, DEVPERIOD_PWR)\n"
    "d50\n"
    "w[4b][01] # ANTAP1_OpenCh(0x01)\n"
    "d50\n"
    "\n"
    "# Speed Cadence\n"
<<<<<<< HEAD
    "w[24][02][00][01] # ANTAP1_AssignCh(0x02)\n"
=======
    "w[42][02][00][01] # ANTAP1_AssignCh(0x02)\n"
>>>>>>> d8e33a4
    "d50\n"
    "w[51][02][00][00][79][00] # ANTAP1_SetChId(0x02,DEVTYPE_BIKE)\n"
    "d50\n"
    "w[44][02][1e] # ANTAP1_SetSearchTimeout(0x02)\n"
    "d50\n"
    "w[45][02][39] # ANTAP1_SetChRFFreq(0x02)\n"
    "d50\n"
    "w[43][02][96][1f] # ANTAP1_SetChPeriod(0x02, DEVPERIOD_BIKE)\n"
    "d50\n"
    "w[4b][02] # ANTAP1_OpenCh(0x02)\n"
    "d50\n\n";

#if 0
void ANTAP1_SetSearchTimeout(unsigned char chan)
{
    unsigned char i;
    unsigned char setup[6];

    setup[0] = 0xa4; // SYNC Byte
    setup[1] = 0x02; // LENGTH Byte
    setup[2] = 0x44; // ID Byte
    setup[3] = chan; // Channel
    setup[4] = 0x1e;
    setup[5] = (0xa4^0x02^0x44^chan^0x1e);  // Checksum

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

void ANTAP1_AssignNetwork(unsigned char chan)
{
    unsigned char i;
    unsigned char setup[13];

    setup[0] = 0xa4; //Sync
    setup[1] = 0x09; //Length
    setup[2] = MESG_NETWORK_KEY_ID;
    setup[3] = chan; //chan
    setup[4] = 0xb9;
    setup[5] = 0xa5;
    setup[6] = 0x21;
    setup[7] = 0xfb;
    setup[8] = 0xbd;
    setup[9] = 0x72;
    setup[10] = 0xc3;
    setup[11] = 0x45;
    setup[12] = (0xa4^0x09^MESG_NETWORK_KEY_ID^chan^0xb9^0xa5^0x21^0xfb^0xbd^0x72^0xc3^0x45);

    for(i = 0 ; i < 13 ; i++)
      putc_serial0(setup[i]);
}


// Assigns CH=0, CH Type=00(RX), Net#=0
void ANTAP1_AssignCh (unsigned char chan)
{
    unsigned char i;
    unsigned char setup[7];

    setup[0] = 0xa4;
    setup[1] = 0x03;
    setup[2] = 0x42;
    setup[3] = chan;        // ChanNum
    setup[4] = chanType;    // ChanType
    setup[5] = netNum;        // NetNum
    setup[6] = (0xa4^0x03^0x42^chan^chanType^netNum);

    for(i = 0 ; i < 7 ; i++)
      putc_serial0(setup[i]);
}

void ANTAP1_SetChRFFreq (unsigned char chan)
{
    unsigned char i;
    unsigned char setup[6];

    setup[0] = 0xa4;
    setup[1] = 0x02;
    setup[2] = 0x45;
    setup[3] = chan;        // ChanNum
    setup[4] = 0x39;        // RF Freq
    setup[5] = (0xa4^0x02^0x45^chan^0x39);

    for(i = 0 ; i < 6 ; i++)
        putc_serial0(setup[i]);

}

void ANTAP1_SetChPeriod (unsigned char chan, unsigned char device)
{
    unsigned char i;
    unsigned char setup[7];

    setup[0] = 0xa4;
    setup[1] = 0x03;
    setup[2] = 0x43;
    setup[3] = chan;
    setup[4] = device;
    setup[5] = 0x1f;
    setup[6] = (0xa4^0x03^0x43^chan^device^0x1f);

    for(i = 0 ; i < 7 ; i++)
        putc_serial0(setup[i]);

}

// Assigns Device#=0000 (wildcard), Device Type ID=00 (wildcard), Trans Type=00 (wildcard)
void ANTAP1_SetChId (unsigned char chan, unsigned char deviceType)
{
    unsigned char i;
    unsigned char setup[9];

    setup[0] = 0xa4;
    setup[1] = 0x05;
    setup[2] = 0x51;
    setup[3] = chan;
    setup[4] = 0x00;
    setup[5] = 0x00;
    setup[6] = deviceType;
    setup[7] = 0x00;
    setup[8] = (0xa4^0x05^0x51^chan^0x00^0x00^deviceType^0x00);

    for(i = 0 ; i < 9 ; i++)
       putc_serial0(setup[i]);
}

// Opens CH 0
void ANTAP1_OpenCh (unsigned char chan)
{
    unsigned char i;
    unsigned char setup[5];

    setup[0] = 0xa4;
    setup[1] = 0x01;
    setup[2] = 0x4b;
    setup[3] = chan;
    setup[4] = (0xa4^0x01^0x4b^chan);

    for(i = 0 ; i < 5 ; i++)
      putc_serial0(setup[i]);

}

#endif

// Assigns CH=0, RF Freq
void ANTAP1_RequestChanID(unsigned char chan)
{
    unsigned char i;
    unsigned char setup[6];

    setup[0] = 0xa4;
    setup[1] = 0x02;
    setup[2] = 0x4d;
    setup[3] = chan;        // ChanNum
    setup[4] = 0x51;        //Extra Info
    setup[5] = (0xa4^0x02^0x4d^chan^0x51);

    for(i = 0 ; i < 6 ; i++)
        putc_serial0(setup[i]);
}
<<<<<<< HEAD
=======



int16_t read_byte(void) {
    uint8_t buf;
    int c=fat16_read_file(handle, &buf, 1);
    if (c==0)
        return -1;
    return (int16_t)buf;
}

void seek(int32_t offset) {
    fat16_seek_file(handle, &offset, FAT16_SEEK_SET);
}

// actually 1/1024 second counts
uint16_t get_milliseconds(void) {
    uint32_t t;
    uint32_t s;
    do {
        t = CTC;
        s = SEC;
    } while (t != CTC);

    t |= s << 16;

    t >>=1; // this gives 32768 Hz
    t >>=5; // divide by 32 for 1024 Hz

    uint16_t r=t & 0xffff;
    return r;
}

void write_ant(char *ant) {
    int i;
    int l=autoant_antlen(ant);

    statLight(0,ON);

    for(i = 0 ; i < l ; i++)
       putc_serial0(ant[i]);

    putc_serial0(0);
    putc_serial0(0);

    statLight(0,OFF);
}

void flush_ant(void) {
    while (U0LSR & 0x1) U0RBR;
}


#define ANT_MAX_LENGTH (30)
#define ANT_BUF_LEN 32
uint8_t ant_rx_buf[ANT_BUF_LEN];
#define ANT_SYNC_BYTE (0xa4)

uint8_t *receive_ant(int timeout) {
  enum States {ST_WAIT_FOR_SYNC, ST_GET_LENGTH, ST_GET_MESSAGE_ID, ST_GET_DATA, ST_VALIDATE_PACKET};
  static enum States state = ST_WAIT_FOR_SYNC;

  uint16_t t0 = get_milliseconds();
  static int length;
  static uint8_t checksum;
  static int chars_in_buf=0;

  while (1) {
    int16_t time_remaining = t0 + timeout - get_milliseconds();
    if (time_remaining < 0) return NULL;

    if (!(U0LSR & 0x1)) continue;
    uint8_t c=U0RBR;

    switch (state) {
    case ST_WAIT_FOR_SYNC:
      if (c == ANT_SYNC_BYTE) {
	state = ST_GET_LENGTH;
	ant_rx_buf[chars_in_buf++] = c;
	checksum = ANT_SYNC_BYTE;

      }
      break;

    case ST_GET_LENGTH:
      if ((c == 0) || (c > ANT_MAX_LENGTH)) {
	chars_in_buf=0;
	state = ST_WAIT_FOR_SYNC;
      }
      else {
	ant_rx_buf[chars_in_buf++] = c;
	checksum ^= c;
	length = chars_in_buf+c+1;
	state = ST_GET_MESSAGE_ID;
      }
      break;

    case ST_GET_MESSAGE_ID:
      ant_rx_buf[chars_in_buf++] = c;
      checksum ^= c;
      state = ST_GET_DATA;
      break;

    case ST_GET_DATA:
      ant_rx_buf[chars_in_buf++] = c;
      checksum ^= c;
      if (chars_in_buf >= length){
	state = ST_VALIDATE_PACKET;
      }
      break;

    case ST_VALIDATE_PACKET:
      ant_rx_buf[chars_in_buf++] = c;

      if (checksum == c){
	chars_in_buf=0; // ready for next time
	state = ST_WAIT_FOR_SYNC;
	return ant_rx_buf;
      }
      // chksum fail. start looking for message again
      chars_in_buf=0;
      state = ST_WAIT_FOR_SYNC;
      break;
    }
  }
}

void write_out(const uint8_t *c) {
    fat16_write_file(out_handle,c,strlen(c));
    sd_raw_sync();
}

void warn_message(int warnno, int linenumber, int pos) {
    char output_string[100];

    string_printf(output_string,
                  "At line %d byte %d : #%d (%s)\n",
                  linenumber, pos, warnno,
                  autoant_interpret_warning(warnno));
    output_string[99]=0;

    write_out(output_string);
}
>>>>>>> d8e33a4

autoant_ops_t ops={
    .receive_ant_message=receive_ant,
    .send_ant_message=write_ant,
    .get_char=read_byte,
    .seek=seek,
    .write=write_out,
    .warn=warn_message,
    .get_milliseconds=get_milliseconds,
    .flush_ant=flush_ant
};


int16_t read_byte(void) {
    uint8_t buf;
    int c=fat16_read_file(handle, &buf, 1);
    if (c==0)
        return -1;
    return (int16_t)buf;
}

void seek(int32_t offset) {
    fat16_seek_file(handle, &offset, FAT16_SEEK_SET);
}

// actually 1/1024 second counts
uint16_t get_milliseconds(void) {
    uint32_t t;
    uint32_t s;
    do {
        t = CTC;
        s = SEC;
    } while (t != CTC);

    t |= s << 16;

    t >>=1; // this gives 32768 Hz
    t >>=5; // divide by 32 for 1024 Hz

    uint16_t r=t & 0xffff;
    return r;
}

void write_ant(char *ant) {
    int i;
    int l=autoant_antlen(ant);

    statLight(0,ON);

    for(i = 0 ; i < l ; i++)
       putc_serial0(ant[i]);

    putc_serial0(0);
    putc_serial0(0);

    statLight(0,OFF);
}

void flush_ant(void) {
    while (U0LSR & 0x1) U0RBR;
}


#define ANT_MAX_LENGTH (30)
#define ANT_BUF_LEN 32
uint8_t ant_rx_buf[ANT_BUF_LEN];
#define ANT_SYNC_BYTE (0xa4)

uint8_t *receive_ant(int timeout) {
  enum States {ST_WAIT_FOR_SYNC, ST_GET_LENGTH, ST_GET_MESSAGE_ID, ST_GET_DATA, ST_VALIDATE_PACKET};
  static enum States state = ST_WAIT_FOR_SYNC;

  uint16_t t0 = get_milliseconds();
  static int length;
  static uint8_t checksum;
  static int chars_in_buf=0;

  while (1) {
    int16_t time_remaining = t0 + timeout - get_milliseconds();
    if (time_remaining < 0) return NULL;

    if (!(U0LSR & 0x1)) continue;
    uint8_t c=U0RBR;

    switch (state) {
    case ST_WAIT_FOR_SYNC:
      if (c == ANT_SYNC_BYTE) {
	state = ST_GET_LENGTH;
	ant_rx_buf[chars_in_buf++] = c;
	checksum = ANT_SYNC_BYTE;

      }
      break;

    case ST_GET_LENGTH:
      if ((c == 0) || (c > ANT_MAX_LENGTH)) {
	chars_in_buf=0;
	state = ST_WAIT_FOR_SYNC;
      }
      else {
	ant_rx_buf[chars_in_buf++] = c;
	checksum ^= c;
	length = chars_in_buf+c+1;
	state = ST_GET_MESSAGE_ID;
      }
      break;

    case ST_GET_MESSAGE_ID:
      ant_rx_buf[chars_in_buf++] = c;
      checksum ^= c;
      state = ST_GET_DATA;
      break;

    case ST_GET_DATA:
      ant_rx_buf[chars_in_buf++] = c;
      checksum ^= c;
      if (chars_in_buf >= length){
	state = ST_VALIDATE_PACKET;
      }
      break;

    case ST_VALIDATE_PACKET:
      ant_rx_buf[chars_in_buf++] = c;

      if (checksum == c){
	chars_in_buf=0; // ready for next time
	state = ST_WAIT_FOR_SYNC;
	return ant_rx_buf;
      }
      // chksum fail. start looking for message again
      chars_in_buf=0;
      state = ST_WAIT_FOR_SYNC;
      break;
    }
  }
}

void write_out(const uint8_t *c) {
    fat16_write_file(out_handle,c,strlen(c));
    sd_raw_sync();
}

void warn_message(int warnno, int linenumber, int pos) {
    char output_string[100];

    string_printf(output_string,
                  "At line %d byte %d : #%d (%s)\n",
                  linenumber, pos, warnno,
                  autoant_interpret_warning(warnno));
    output_string[99]=0;

    write_out(output_string);
}

autoant_ops_t ops={
    .receive_ant_message=receive_ant,
    .send_ant_message=write_ant,
    .get_char=read_byte,
    .seek=seek,
    .write=write_out,
    .warn=warn_message,
    .get_milliseconds=get_milliseconds,
    .flush_ant=flush_ant
};

/*******************************************************
 *                      MAIN
 ******************************************************/

int main (void)
{
    int i;
    char name[32];
    int count = 0;

    seen[0] = FALSE;
    seen[1] = FALSE;
    seen[2] = FALSE;
    seen[3] = FALSE;

    enableFIQ();

    Initialize();

    fat_initialize();

    set_time();

    setup_uart0(baud, 0);

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

    out_handle = root_open("out.txt");
    if (!out_handle)
        out_handle = root_open_new("out.txt");

    int32_t zero=0;
    fat16_seek_file(out_handle,&zero,FAT16_SEEK_END);
    fat16_write_file(out_handle,"startup-----------\n",
                         strlen("startup-----------\n"));

    handle = root_open("init.ant");
    if (0==handle) {
        fat16_write_file(out_handle,
                         "no init file found\n",
                         strlen("no init file found\n"));
        handle = root_open_new("init.ant");
        fat16_write_file(handle,
                         antap1_config,
                         strlen(antap1_config));
        autoant_exec(&ops);
        fat16_close_file(handle);
        handle = root_open("init.ant");
    } else {
        fat16_write_file(out_handle,
                         "found init file\n",
                         strlen("found init file\n"));
    }

<<<<<<< HEAD
=======
    {
        char baudrate_buffer[200];

        int l=fat16_read_file(handle, &baudrate_buffer, 200);

        for (i=0; i<l; i++) {
            if (0 == memcmp(baudrate_buffer+i, "baudrate", strlen("baudrate"))) {
                i += strlen("baudrate");
                while ((i+4<l) && isspace(baudrate_buffer[i]))
                    i++;
                if (baudrate_buffer[i] != '=') 
                    break;
                i++;
                
                int new_baudrate=atoi(baudrate_buffer+i);
                setup_uart0(new_baudrate, 0);                
                
            }
        }
    }

    seek(0);


>>>>>>> d8e33a4
    autoant_exec(&ops);
    fat16_close_file(handle);
    fat16_close_file(out_handle);

    count++;
    string_printf(name,"ANT%02d.gce",count);
    while(root_file_exists(name))
    {
        count++;
        if(count == 250)
        {            while(1)
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

    set_time();

    mode_0();

    return 0;
}




//Set the time
void get_time(void)
{
    int foo = YEAR;
    foo = MONTH;
    foo = DOM;
    ts.hour = HOUR;
    ts.minute = MIN;
    ts.second = SEC;
}


//Set the time
void set_time(void)
{
  //Turn on timer 1, 60MHz by default
    T1TCR=0x01; // enable timer1

  CCR=0x13;  //Turn off RTC
  YEAR=0;
  MONTH=0;
  DOM=0;
  DOW=0;
  DOY=0;
  HOUR=0;
  MIN=0;
  SEC=0;
  CCR=0x11;     //Turn RTC back on
}



/*******************************************************
 *                   Initialize
 ******************************************************/

#define PLOCK 0x400

void Initialize(void)
{
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
    unsigned char current=U0RBR;

    if(RX_in < 512)
    {
        RX_array1[RX_in] = current;
        RX_in++;

        if(RX_in == 512) log_array1 = 1;
    }
    else if(RX_in >= 512)
    {
        RX_array2[RX_in-512] = current;
        RX_in++;

        if(RX_in == 1024)
        {
            log_array2 = 1;
            RX_in = 0;
        }
    }


    temp = U0IIR; // Have to read this to clear the interrupt

    VICVectAddr = 0;

    if(parseANT(current))
    {
        get_time(); //It's the end of a MESG, get the time.
        add_time_stamp(); //Add the time to the end of the MESG.
        if(isBroadCast == TRUE && currentChannel>=0 && seen[currentChannel] == FALSE)
        {
            seen[currentChannel] = TRUE;
            ANTAP1_RequestChanID(currentChannel);
        }
    }
}

void add_time_stamp(void)
{

   //The ugliest code you ever did see..
    for(int i=0; i < 3; i++)
	{
            switch(i)
            {
            case 0:
            if(RX_in < 512)
            {
                RX_array1[RX_in] = ts.hour;
                RX_in ++;

                if(RX_in == 512)
                {
                    log_array1 = 1;
                    must_we_write();
                }
            }
            else if(RX_in >= 512)
            {
                RX_array2[RX_in - 512] = ts.hour;
                RX_in ++;

                if(RX_in == 1024)
                {
                    log_array2 = 1;
                    RX_in = 0;
                    must_we_write();
                }
            }
            break;
            case 1:
            if(RX_in < 512)
            {
                RX_array1[RX_in] = ts.minute;
                RX_in ++;

                if(RX_in == 512)
                {
                    log_array1 = 1;
                    must_we_write();
                }

            }
            else if(RX_in >= 512)
            {
                RX_array2[RX_in - 512] = ts.minute;
                RX_in ++;

                if(RX_in == 1024)
                {
                    log_array2 = 1;
                    RX_in = 0;
                    must_we_write();
                }

            }
            break;
            case 2:
            if(RX_in < 512)
            {
                RX_array1[RX_in] = ts.second;
                RX_in ++;

                if(RX_in == 512)
                {
                    log_array1 = 1;
                    must_we_write();
                }

            }
            else if(RX_in >= 512)
            {
                RX_array2[RX_in - 512] = ts.second;
                RX_in ++;

                if(RX_in == 1024)
                {
                    log_array2 = 1;
                    RX_in = 0;
                    must_we_write();
                }
            }
            break;
         }
      }
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

<<<<<<< HEAD
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
=======
    int32_t bauddiv=3686400 / baud;

    U0DLM = (bauddiv >> 8) & 0xff;
    U0DLL = bauddiv & 0xff;
>>>>>>> d8e33a4

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
    // ANTAP1_Config();
    mode_action();
}


void must_we_write(void)
{
    int j;

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
}


void mode_action(void)
{
    int j;
    while(1)
    {
        must_we_write(); //Check to see if the buffer if ready to be written

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
     else        // bank 1
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

}

void fat_initialize(void)
{
    if(!sd_raw_init())
    {
        while(1);
    }

    openroot();

}

void delay_ms(int count)
{
    int i;
    count *= 10000;
    for(i = 0; i < count; i++)
        asm volatile ("nop");
}

int parseANT(unsigned char chr)
{

    if ((chr == MESG_TX_SYNC) && (inMsg == FALSE))
    {
        msgN = 0; // Always reset msg count if we get a sync
        inMsg = TRUE;
        currentChannel=-1;
        msgN++;
    }
    else if (msgN == 1)
    {
        msgN++;
        size = chr;
    }
    else if(msgN == 2)
    {
      if(chr == MESG_BROADCAST_DATA_ID) {
        isBroadCast = TRUE;
      } else {
        isBroadCast = FALSE;
      }
      msgN++;
    }
    else if (msgN == 3) {
      currentChannel=(int) chr; // this has to be 0x00,0x01,0x02,0x03 so okay?
      msgN++;
    }
    else if (msgN == (size + 3)) // sync, size, checksum
    {
        //Write the time.
        inMsg = FALSE;
        msgN = 0;
        return 1; //We are at the end of the message
    }
    else if(inMsg == TRUE)
    {
        msgN++;
    }

    return 0;
}
