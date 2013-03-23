#ifndef AUTOANT_H
#define AUTOANT_H

#include <stdint.h>

typedef struct {

  void (*seek)(int); /* looping is accomplished by seek()ing
			backwards */

  int16_t (*get_char)(void);  /* fgetc() or similar, <0 for EOF */

  void (*send_ant_message)(uint8_t *); /* sends ANT message.  Message
                                          is ready to send including
                                          prepended 0xa4 start byte
                                          and appended checksum. */

  uint8_t *(*receive_ant_message)(int); /* receives ANT message. The
					   length is computed from the
					   length field in the
					   message.  Timeout in
					   milliseconds.  Return NULL
					   for timeout. */

  void (*warn)(int warnno, int lineno, int pos); /* returns warning
                                                    number.  Interpret
                                                    with
                                                    autoant_interpret_warning()
                                                    function. */

  void (*write)(char *); /* prints (or otherwise deals with) an output
			  message.  Message is null-terminated
			  ASCII. */

  void (*pause)(void); /* wait for something to happen -- corresponds
			  to "p" command */

  void (*delay_ms)(void); /* wait for up to one millisecond.  If this
			     returns too soon, it will get called
			     repeatedly. */

  uint16_t (*get_milliseconds)(void); /* count of milliseconds for delay
					 purposes */

  void (*flush_ant)(void); /* flush any buffered ANT messages */

} autoant_ops_t;

int autoant_antlen(uint8_t *c); /* gives the length of an ant buffer,
				   based on the len byte.  Buffer
				   starts with 0xa4 sync byte. */
char *autoant_interpret_warning(int warnno); /* translate a warning
                                                number into English
                                                (ASCII,
                                                null-terminated)
                                                warning string. */

int autoant_exec(autoant_ops_t *ops); /* Call once to run script until
                                         EOF */

enum { AUTOANT_WARN_LOOP_NESTING = -1,
       AUTOANT_WARN_NO_ITERATION = -2,
       AUTOANT_WARN_STACK_OVERFLOW = -3,
       AUTOANT_WARN_TRUNCATED_LINE = -4,
       AUTOANT_WARN_GARBAGE_IN_BRACKETS = -5,
       AUTOANT_WARN_SIZE_EXCEEDED = -6 }; // see autoant_interpret_warning()

#endif
