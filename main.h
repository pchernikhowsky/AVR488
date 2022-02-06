/*
 * AVR488
 *
 * main.h
 *
 * Created: 28-Dec-2021 7:19:56 PM
 *  
 */


#ifndef MAIN_H_
#define MAIN_H_

// GPIB command bytes
#define CMD_DCL 0x14
#define CMD_UNL 0x3f
#define CMD_UNT 0x5f
#define CMD_GET 0x08
#define CMD_SDC 0x04
#define CMD_LLO 0x11
#define CMD_GTL 0x01
#define CMD_SPE 0x18
#define CMD_SPD 0x19

// ++ command decoder macro
#define IS_CMD(cmd_name) (strncasecmp_P(buf+2,PSTR(cmd_name),(sizeof cmd_name)-1)==0)

// Text input buffer size
#define BUFSIZE 254	// Size of internal line buffer uart_gets()

// EEPROM variables
#define VALID_EEPROM_CODE 0xaa

extern void _wdt_reset(void); // allow other files to reset the watchdog timer

#endif /* MAIN_H_ */