/*
  auto-ant script parser / interpreter

  Copyright (c) 2013, Quarq Technology / SRAM LLC
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

  Neither the name of Quarq Technology nor SRAM LLC nor the names of
  its contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

*/

#include "autoant.h"

#include <ctype.h> // for isspace() macro
#include <string.h> // for strlen(), memcmp()
//#include <stdio.h> // debugging

#define AUTOANT_MAX_RECURSION_DEPTH 5
#define AUTOANT_MAX_LINE_LEN 240

static int autoant_delay;
 
typedef struct {
    int position; // bytes into file
    int linenumber;
    int iterations_left;
} autoant_stack_frame_t;

static autoant_stack_frame_t autoant_stack[AUTOANT_MAX_RECURSION_DEPTH];
static int stack_depth;

static autoant_ops_t *op;

static void autoant_init_parser(void) {
    autoant_delay=3000;
    stack_depth=0;
    autoant_stack[stack_depth].position=0;
    autoant_stack[stack_depth].linenumber=0;
    autoant_stack[stack_depth].iterations_left=0;  
}

char *autoant_interpret_warning(int warnno) {
   
    switch (warnno) {
    case AUTOANT_WARN_SIZE_EXCEEDED:
        return "Buffer size exceeded";
    case AUTOANT_WARN_LOOP_NESTING:
        return "loopend statement without previous loop() statement";
    case AUTOANT_WARN_NO_ITERATION:
        return "No iterations specified in loop() statement";
    case AUTOANT_WARN_STACK_OVERFLOW:
        return "Too many nested loop() statements";
    case AUTOANT_WARN_TRUNCATED_LINE:
        return "Line exceeded compiled-in limit";
    case AUTOANT_WARN_GARBAGE_IN_BRACKETS:
        return "Found unknown (non-hexadecimal) character within []";
    default:
        return "Unknown warning";
    }
}

#define WARN(warnno) do {                                             \
        if (op->warn) op->warn(warnno,                                \
                               autoant_stack[stack_depth].linenumber, \
                               autoant_stack[stack_depth].position);  \
    } while (0);					
    
#define OUTPUT(msg) do {			\
        if (op->write) op->write(msg);		\
    } while (0);					

#define LINE_STARTSWITH(val) 0==memcmp(line,val,strlen(val))

// define the overall length of Ant message based on length byte
int autoant_antlen(uint8_t *c) {
    // header+length byte + id + data + checksum
    return 1+2+c[1]+1;
}

static int get_pos_integer(char *line) {
    int ret=0;
    int found_num=0;

    char *c=line;
  
    while ((*c) && isspace(*c))
        c++;
  
    while (1) {
        switch (*c) {
        case '0'...'9':
            found_num = 1;
            ret *= 10;
            ret += (*c-'0');
            break;
        default:
            if (0==found_num) ret=-1;
            return ret;
        }
        c++;
    }
}

#define MAX_BUFFER_LEN 32
static uint8_t char_array_readout[MAX_BUFFER_LEN];

static uint8_t *get_hex_char_array(char *line) {
    uint8_t *c=(uint8_t *)line;
    uint16_t current;
    int len=0;
    uint8_t checksum=0;

    char_array_readout[len++]=0xa4; checksum ^= 0xa4;
    len++; // skip length byte, fill in later

    while (1) {
        while ((*c) && (*c != '[')) {
            c++;
        }
        if (*c=='\0') goto done;
        // now *c is '[' and we're ready to parse hex.

        current=0;
        while (*c != ']') {
            //printf("char '%c' current %02x\n",*c,current); 
            switch (*c) {
            case 0:
                goto done;
            case 'a'...'f':
                current <<= 4;
                current += 10+(*c-'a');
                break;
            case 'A'...'F':
                current <<= 4;
                current += 10+(*c-'A');
                break;
            case '0'...'9':
                current <<= 4;
                current += (*c-'0');
                break;
            case '[':
                break;
            default:
                //printf("Skipping char '%c'\n",*c);
                WARN(AUTOANT_WARN_GARBAGE_IN_BRACKETS);
                break;
            }
            c++;
        }
        //printf("Got char %02x\n",current);
        char_array_readout[len++]=current; checksum ^= current;

        if (len+1==MAX_BUFFER_LEN) {
            WARN(AUTOANT_WARN_SIZE_EXCEEDED);                
            return NULL;
        }
    }

 done:
    if (len==0) {
        return NULL;    
    } else {
        char_array_readout[1]=(len-3); checksum ^= (len-3);
        char_array_readout[len++]=checksum;
        return char_array_readout;
    }
}

// returns -1 for end processing, 0 for continue. 

static int autoant_run_line(char *line, int len) {
 
    //printf("Processing: %s\n", line);
 
    /* skip whitespace */
    char *end_char=line+len; // actually, one past last character
  
    while (line < end_char) {
        if (isspace(*line)) {
            line++;
        } else {
            break;
        }    
    }
  
    /* remove comments off the end */
    char *x=line;
    while (x<end_char) {
        if (*x=='#') 
            end_char=x;
        x++;
    }

    *end_char=0; // null term, just in case

    //printf("actual line %d (iteration %d): '%s'\n",
    //autoant_stack[stack_depth].linenumber,
    //	 autoant_stack[stack_depth].iterations_left, line);

    if (line==end_char) 
        return 0; // blank line, skip
      
    if (LINE_STARTSWITH("o")) {
        OUTPUT(line+1);
    } else if (LINE_STARTSWITH("d")) {
        int d=get_pos_integer(line+1);

        uint16_t timeout = op->get_milliseconds()+d;
    
        int16_t timeremain; 

        while ((timeremain = timeout - op->get_milliseconds())>0) {
            if (op->delay_ms) op->delay_ms();
        }
        if (op->flush_ant) op->flush_ant();

    } else if (LINE_STARTSWITH("w")) {
        uint8_t *c=get_hex_char_array(line+1);
        op->send_ant_message(c);
    } else if (LINE_STARTSWITH("r")) {
        line++;
        int hard_fail=(*line =='!');
        if (hard_fail) line++;
    
        int d=get_pos_integer(line);
        if (d<0) d=autoant_delay;

        uint8_t *c=get_hex_char_array(line);

        if (c==NULL) {
            autoant_delay=d;
            return 0;
        }

        uint8_t *c2;

        uint16_t timeout = op->get_milliseconds()+d;
    
        int16_t timeremain; 

        while ((timeremain = timeout - op->get_milliseconds())>0) {
            c2=op->receive_ant_message(timeremain);
            if (c2 && memcmp(c2,c,autoant_antlen(c))) break;
        }

        if (c2==NULL) {
            if (hard_fail) return -1;
        }

    } else if (LINE_STARTSWITH("pauseBreak")) {
        if (op->pause) op->pause();
    } else if (LINE_STARTSWITH("loopend")) {
        if (stack_depth==0) {
            WARN(AUTOANT_WARN_LOOP_NESTING);
            return -1;
        }
        if (autoant_stack[stack_depth].iterations_left) {
            op->seek(autoant_stack[stack_depth-1].position);

            autoant_stack[stack_depth].position=autoant_stack[stack_depth-1].position;
            autoant_stack[stack_depth].linenumber=autoant_stack[stack_depth-1].linenumber;
            autoant_stack[stack_depth].iterations_left -= 1;
        } else {
            autoant_stack[stack_depth-1].position=autoant_stack[stack_depth].position;
            autoant_stack[stack_depth-1].linenumber=autoant_stack[stack_depth].linenumber;
            stack_depth-=1;
        }
     
    } else if (LINE_STARTSWITH("loop")) {
        while (*line)
            if (*line++=='(') break;

        int iterations=get_pos_integer(line);
        if (iterations<1) {
            WARN(AUTOANT_WARN_NO_ITERATION);
            return -1;
        }
        if ((stack_depth+1) >=AUTOANT_MAX_RECURSION_DEPTH) {
            WARN(AUTOANT_WARN_STACK_OVERFLOW);
            return -1;
        }
        autoant_stack[stack_depth+1].position=autoant_stack[stack_depth].position;
        autoant_stack[stack_depth+1].linenumber=autoant_stack[stack_depth].linenumber;
        autoant_stack[stack_depth+1].iterations_left=iterations-1;

        stack_depth++;
    }

    return 0;
}

static char linebuf[AUTOANT_MAX_LINE_LEN];


// reads a line and runs it

/* returns: -1 for runtime error
   -2 for parse error
   0 for EOF, line was OK.
   1 for OK, not EOF */

static int autoant_read_line(void) {

    int len=0;
    int c;
    do {

        c=op->get_char();
        autoant_stack[stack_depth].position++;

        if (c<0) break;
    
        if (len<AUTOANT_MAX_LINE_LEN) {
            linebuf[len++]=c;
        }

    } while (c != '\n');

    autoant_stack[stack_depth].linenumber++;

    if (len == AUTOANT_MAX_LINE_LEN) { WARN(AUTOANT_WARN_TRUNCATED_LINE); len--; }
    
    linebuf[len]='\0'; // overwrite \n with \0 for c-friendly null terminator
  
    int ret=autoant_run_line(linebuf, len);

    if ((c >= 0) && (ret==0)) ret=1;
    return ret;
}

int autoant_exec(autoant_ops_t *ops) {
    op=ops;
    op->seek(0);
    int ret;
    autoant_init_parser();
  
    do {
        ret=autoant_read_line();
    } while (ret == 1);

    return ret;
}
