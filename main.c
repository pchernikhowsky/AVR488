/*
 * AVR488
 *
 * Created: 28-Dec-2021 7:14:34 PM
 *
 * Prologix compatible(ish) GPIB to USB Adapter based on the Atmel/Microchip ATmega168/328 devices
 *
 * Inspired by code from
 * (https://github.com/Galvant/gpibusb-firmware) and AR488 (https://github.com/Twilight-Logic/AR488)
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#define F_CPU 8000000UL	// internal RC oscillator frequency = 8 MHz
#define BAUD 38400 // USART baud rate
#define VERSION "1.0" // firmware version

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "main.h"
#include "uart.h"
#include "pinio.h"

uint8_t mcusr __attribute__((section(".noinit"))); // save the MCUSR reset cause

FILE usart_str = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE); // Use streams for character output to usart only
char cmd_buf[5], buf[BUFSIZE];

bool eoiUse; // whether we are using EOI to signal end of message from instrument
bool debug; // enable or disable read&write error messages
bool eot_enable; // whether to append eot_char after EOI
char eot_char; // end of transmission character added after EOI
char eos_code; // end of transmission code to end a transmission to the instrument
char eos_string[3]; // characters to end a transmission to the instrument

// Variables timeout monitoring during transmit/receive
unsigned int timeout = 1000; // used for send/receive timeout monitoring - default timeout is 1000 milliseconds
volatile unsigned int ticks = 0; // incremented in ISR (so use volatile declaration)


/***********************************************************************************************************
*
* Start of helper subroutines
*
************************************************************************************************************/

/*
 * TIMER1 interrupt handler
 */
ISR(TIMER1_COMPB_vect)
{
	++ticks;	// increment tick counter every millisecond
}


/*
* Reset the watchdog timer and toggle the status LED
*/
void _wdt_reset(void)
{
	wdt_reset();
	PIND = 1<<(LED_STATUS & 0x7f); // strip the high bit from the define
}


/*
 * Puts all the GPIB pins into their correct initial states.
 */
void prep_gpib_pins()
{
	output_low(TE); // Disable Talk Enable for data and handshake lines
	output_high(PE); // Enable Pullups for bus transceivers
	output_low(DC); // Allows transmit on ATN/REN/IFC and listen on SRQ

	// tri-state data pins
	DDRB = 0;	 // all pins input
	PORTB = 0xff; // enable pullups

	DEASSERT(ATN);
	FLOAT(EOI);
	FLOAT(DAV);
	ASSERT(NRFD);
	ASSERT(NDAC);
	DEASSERT(IFC);
	FLOAT(SRQ);
	ASSERT(REN);
}


/*
 * Send a multibyte GPIB data string (command or data).
 */
char _gpib_write(char *bytes, int length, bool attention)
{
    /*
    * Write a string of bytes to the bus
    * bytes: array containing characters to be written
    * length: number of bytes to write, 0 if not known.
    * attention: 1 if this is a GPIB command, 0 for data
	*
	* Returns true if there was an error
    */
	char a; // Storage variable for the current character
	int i; // Loop counter variable

	if(attention) // If byte is a GPIB bus command
		ASSERT(ATN); // Assert the ATN line, informing all that this is a command byte

	if(length==0) // If the length was unknown
		length = strlen(bytes); // Calculate the number of bytes to be sent

	output_high(TE); // Enable talking
	DEASSERT(EOI);
	DEASSERT(DAV);
	FLOAT(NRFD); // Let talkers control NRFD and NDAC
	FLOAT(NDAC);

	if (debug == 1)
		printf_P(PSTR("NDAC:%u / NRFD:%u\n\r"), input(NDAC), input(NRFD));

	for(i = 0;i < length;i++) //Loop through each character, write to bus
	{
		a = bytes[i]; // Character to send

		// Wait for listeners to be ready for data (NRFD should be high and NDAC low)
		ticks = 0;
		while((!input(NRFD) || input(NDAC)) && (ticks <= timeout))
		{
		    _wdt_reset();
			if(ticks >= timeout)
			{
			    if (debug == 1)
				    printf_P(PSTR("Timeout: Waiting for NRFD to go high and NDAC to go low while writing\n\r"));
			    prep_gpib_pins();
				return 1;
			}
		}

		// Enable port B for output and put the byte on the data lines using negative logic
		if (debug == 1)
		{
			if (attention)
				printf_P(PSTR("Writing command byte: 0x%02x\n\r"), a);
			else
				printf_P(PSTR("Writing data byte: %c 0x%02x\n\r"), isprint(a)?a:' ', a);
		}
		DDRB = 0xff;
		PORTB = ~a;

		if((i==length-1) && eoiUse && !attention)
		{
			ASSERT(EOI); // If last byte in string assert EOI
		    if (debug == 1)
			    printf_P(PSTR("Asserting EOI\n\r"));
		}
		_delay_us(150); // delay to allow lines to settle
		ASSERT(DAV); // Inform listeners that the data is ready to be read
		_delay_us(150); // delay to allow lines to settle

		// Wait for NDAC to go high, all listeners have accepted the byte
		ticks = 0;
		while(!(input(NDAC)) && (ticks <= timeout))
		{
		    _wdt_reset();
			if(ticks >= timeout)
			{
			    if (debug == 1)
			        printf_P(PSTR("Timeout: Waiting for NDAC to go high while writing\n\r"));
			    prep_gpib_pins();
				return 1;
			}
		}

		DEASSERT(DAV); // Byte has been accepted by all, indicate the byte is no longer valid

	} // Finished outputting all bytes to listeners

	DDRB = 0; // Set Port B back to all inputs (pull-ups are still active)
	output_low(TE); // Disable talking on data lines

	if(attention) // If byte was a GPIB command byte release ATN line
		DEASSERT(ATN);

	FLOAT(DAV);
	FLOAT(EOI);
	DEASSERT(NDAC);
	DEASSERT(NRFD);

	return 0;
}


/*
 * Send a multibyte GPIB command (ATN line asserted).
 */
char gpib_cmd(char *bytes, int length)
{
	if (debug == 1)
		printf_P(PSTR("gpib_cmd(): %d bytes\n\r"), length);

    // Write a GPIB CMD byte to the bus
	return _gpib_write(bytes, length, 1);
}


/*
 * Make ourselves the bus controller.
 */
char gpib_controller_assign(void)
{
	ASSERT(IFC); // Assert interface clear. Resets bus and makes it controller in charge.
	_wdt_reset();
	_delay_us(150);
	FLOAT(IFC); // Finishing clearing interface

	ASSERT(REN); // Put all connected devices into "remote" mode
	cmd_buf[0] = CMD_DCL;
	return gpib_cmd(cmd_buf, 1); // Send GPIB DCL command, clear all devices on bus
}


/*
 * Send a multibyte GPIB data string (ATN line not asserted).
 */
char gpib_write(char *bytes, int length)
{
	if (debug == 1)
		printf_P(PSTR("gpib_write(): \"%s\"\n\r"),bytes);

    // Write a GPIB data string to the bus
	return _gpib_write(bytes, length, 0);
}


/*
 * Receive a single byte from the GPIB bus.
 */
char gpib_receive(char *byte)
{
	char a = 0; // Storage for received character
	char eoiStatus; // Returns 0x00 or 0x01 depending on status of EOI line or 0xff for error

	// Let the talker control the data valid line
	FLOAT(DAV);

	// Raise NRFD, telling the talker we are ready for the byte
	DEASSERT(NRFD);

	// Assert NDAC informing the talker we have not accepted the byte yet
	ASSERT(NDAC);

	// Wait for DAV to go low (talker informing us the byte is ready)
    ticks = 0;
	while (input(DAV) && (ticks <= timeout))
	{
	    _wdt_reset();
		if (ticks >= timeout)
		{
		    if (debug == 1)
			    printf_P(PSTR("Timeout: Waiting for DAV to go low while reading\n\r"));
		    prep_gpib_pins();
			return 0xff;
		}
	}

	// Assert NRFD, informing talker to not change the data lines
	ASSERT(NRFD);

	// Read port B, where the data lines are connected
	a = ~PINB; // Flip all bits since GPIB uses negative logic.
	eoiStatus = input(EOI);
	_delay_us(150);

	if (debug == 1)
		printf_P(PSTR("Read byte: %c 0x%02x\n\r"), isprint(a)?a:' ', a);

	// Deassert NDAC, informing talker that we have accepted the byte
	FLOAT(NDAC);
	_delay_us(150);

	// Wait for DAV to go high (talker knows that we have read the byte)
    ticks = 0;
	while(!(input(DAV)) && (ticks<=timeout) )
	{
	    _wdt_reset();
		if(ticks >= timeout)
		{
		    if (debug == 1)
			    printf_P(PSTR("Timeout: Waiting for DAV to go high while reading\n\r"));
		    prep_gpib_pins();
			return 0xff;
		}
	}

	// Prep for next byte, we have not accepted anything
	ASSERT(NDAC);

	if (debug == 1)
		printf_P(PSTR("EOI: %x\n\r"), (eoiStatus>0)?0:1); // invert EOI (negative logic)

	*byte = a;

	return eoiStatus;
}


/*
 * Receive a multiple bytes from the GPIB bus until either EOI or timeout.
 */
char gpib_read(bool read_until_eoi, uint8_t partnerAddress)
{
	char readCharacter,eoiStatus;
	char errorFound = 0;
	bool reading_done = false;

	if (debug == 1)
		printf_P(PSTR("gpib_read start - read_until_eoi=%d\n\r"), read_until_eoi);

	cmd_buf[0] = CMD_UNT;
	cmd_buf[1] = CMD_UNL;
	cmd_buf[2] = partnerAddress + 0x40;
	errorFound = gpib_cmd(cmd_buf, 3);
	if (errorFound)
		return 1;

	if (debug == 1)
		printf_P(PSTR("gpib_read loop start\n\r"));

	if (read_until_eoi == 1)
	{
		do
		{
			eoiStatus = gpib_receive(&readCharacter); // eoiStatus is line level
			_wdt_reset();
			if (!debug)
				putchar(readCharacter);
			if(eoiStatus == 0xff)
				return 1;
		} while (eoiStatus);
		if ( eot_enable)
			putchar(eot_char);
	}
	else
	{
		do
		{
			eoiStatus = gpib_receive(&readCharacter);
			_wdt_reset();
			if (!debug)
				putchar(readCharacter);
			if (eoiStatus==0xff)
				return 1;
			if (eos_code != 3)
			{
			    if (readCharacter == eos_string[0]) // Check for EOM chars
			        reading_done = true;
			    if (readCharacter == eos_string[1])
		            reading_done = true;
			}
		} while (reading_done == false);
		reading_done = false;
	}

	errorFound = 0;
	// Command all talkers and listeners to stop
	cmd_buf[0] = CMD_UNT;
	cmd_buf[1] = CMD_UNL;
	errorFound = gpib_cmd(cmd_buf, 2);

	if (debug == 1)
		printf_P(PSTR("gpib_read end\n\r"));

	return errorFound;
}


/*
* Address the currently specified GPIB address (as set by the ++addr cmd) to listen
*/
char addressTarget(int address)
{
    cmd_buf[0] = CMD_UNT; // everyone stop talking
    cmd_buf[1] = CMD_UNL; // everyone stop listening
    cmd_buf[2] = address + 0x20; // tell addressed device to listen
    return gpib_cmd(cmd_buf, 3);
}


/*
* Fetch (and invert - due to negative logic) the state of the SRQ bus line
*/
bool srq_state(void)
{
    return !input(SRQ);	// invert due to negative logic
}


/*
* Fetch the instrument status byte using a serial poll command
*/
void serial_poll(int address)
{
    char error = 0;
    char status_byte;
	char buffer [10];

    cmd_buf[0] = CMD_SPE; // enable serial poll
	cmd_buf[1] = address + 0x40;
    error = gpib_cmd(cmd_buf, 2);
    if ( gpib_receive(&status_byte) == 0xff)
		error = 1;
    cmd_buf[0] = CMD_SPD; // disable serial poll
	error = error || gpib_cmd(cmd_buf, 1);
	if (!error)
	{
		utoa(status_byte,buffer,2);
	    printf_P(PSTR("%s\n\r"), buffer);
	}
	else
	    printf_P(PSTR("ERROR\n\r"));
}


/*
* Start TIMER1 to generate periodic interrupts (for use in data transmission timeout monitoring)
*/
void init_timer(void)
{
	TCCR1B = 0x00; // stop timer
	TCCR1A = 0x00; // OC1x output compare and PWM not used
	OCR1A = 1000; // count to 1000 - gives a 1.0 ms periodic interrupt
	TCCR1B = 1<<WGM12 | 1<<CS11; // start timer with divide by 8 prescaler and clear count on compare match
	TIMSK1 |= 1<<OCIE1B; // enable the timer interrupt to free run
}


/*
* Read out and reset mcusr early during startup. Also disable WDT if it was left running after reset.
*/
void handle_mcusr(void)
__attribute__((section(".init3")))
__attribute__((naked));
void handle_mcusr(void)
{
	mcusr = MCUSR;
	MCUSR = 0;
	wdt_disable();
}


/*
* Populate the GPIB end of string terminators
*/
void set_eos_string(uint8_t code)
{
    switch (code)
    {
	    case 0:
			eos_code = 0;
			eos_string[0] = '\r';
			eos_string[1] = '\n';
			eos_string[2] = 0x00;
		    break;
	    case 1:
			eos_code = 1;
			eos_string[0] = '\r';
			eos_string[1] = 0x00;
			break;
		case 2:
			eos_code = 2;
			eos_string[0] = '\n';
			eos_string[1] = 0x00;
			break;
		default:
			eos_code = 3;
			eos_string[0] = 0x00;
			break;
    }
}


/*
* Show all valid commands
*/
void print_help(void)
{
	printf_P(PSTR("Available commands (all are preceded by \"++\" and not case sensitive):\n\r"));
	printf_P(PSTR("addr 1-30      Tell controller which instrument to address.\n\r"));
	printf_P(PSTR("addr           Query currently configured instrument address.\n\r"));
	printf_P(PSTR("auto 0|1       Enable (1) or disable (0) read after write.\n\r"));
	printf_P(PSTR("auto           Query read after write setting.\n\r"));
	printf_P(PSTR("clr            Issue device clear.\n\r"));
	printf_P(PSTR("debug 0|1      Enable (1) or disable (0) debug messages.\n\r"));
	printf_P(PSTR("debug          Query current debug level.\n\r"));
	printf_P(PSTR("echo 0|1       Enable (1) or disable (0) echoing of characters received from USB port.\n\r"));
	printf_P(PSTR("echo           Query current echo setting.\n\r"));
	printf_P(PSTR("eoi 0|1        Enable (1) or disable (0) EOI with last byte.\n\r"));
	printf_P(PSTR("eoi            Query current EOI setting.\n\r"));
	printf_P(PSTR("eos 0|1|2|3    EOS terminator — 0:CR+LF, 1:CR, 2:LF, 3:None.\n\r"));
	printf_P(PSTR("eos            Query current EOS setting.\n\r"));
	printf_P(PSTR("eot_char N     End of transmission character ASCII code.\n\r"));
	printf_P(PSTR("eot_char       Query current end of transmission character ASCII code.\n\r"));
	printf_P(PSTR("eot_enable N   End of transmission character enable (1) or disable (0).\n\r"));
	printf_P(PSTR("eot_enable     Query current end of transmission character setting.\n\r"));
	printf_P(PSTR("help           Print help about commands.\n\r"));
	printf_P(PSTR("ifc            Issue interface clear.\n\r"));
	printf_P(PSTR("llo            Disable front panel operation of the currently addressed instrument.\n\r"));
	printf_P(PSTR("loc            Enable front panel operation of the currently addressed instrument.\n\r"));
	printf_P(PSTR("mode 1         Compatibility only command - we are always a controller.\n\r"));
	printf_P(PSTR("read           Read from instrument until timeout.\n\r"));
	printf_P(PSTR("read eoi       Read from instrument until EOI detected or timeout.\n\r"));
	printf_P(PSTR("read_tmo_ms N  Timeout value, in milliseconds, for read and spoll commands.\n\r"));
	printf_P(PSTR("read_tmo_ms    Query current timeout value.\n\r"));
	printf_P(PSTR("rst            Performs power-on reset of the controller.\n\r"));
	printf_P(PSTR("savecfg 1      Save current configuration to EEPROM.\n\r"));
	printf_P(PSTR("spoll [N]      Read status byte by serial polling the instrument.\n\r"));
	printf_P(PSTR("srq            Query status of SRQ line. 0: Not asserted, 1:Asserted.\n\r"));
	printf_P(PSTR("trg [N]        Issue device trigger (GET) to the instrument.\n\r"));
	printf_P(PSTR("ver            Query AVR488 interface version.\n\r"));
	printf_P(PSTR("xdiag          Show current state of AVR IO pins.\n\r"));
}


/***********************************************************************************************************
 *
 * Main routine
 *
 ***********************************************************************************************************/
int main(void)
{
	uint8_t partnerAddress = 0; // GPIB address of instrument we're communicating with
	bool localecho = 1; // whether to echo characters received from the USB port to the user
	bool autoread = 1; // whether to automatically query an instrument (if there's a "?" in a command string)
	
	// some temporary variables
	bool writeError = 0;
	int addrTemp;
	char buffer [10];


	output_high(LED_STATUS); // Turn on the status LED - first sign of life
	_delay_ms(200);

	// Setup the periodic interrupt timer
	init_timer();

	// Setup the watchdog timer with a one second timeout
	wdt_enable(WDTO_1S);

	// enable global interrupts
	sei();

	// setup USB communications and stdio output stream
	uart_init( UART_BAUD_SELECT(BAUD,F_CPU) );
	stdout = &usart_str; // point to the character output routine for stdio output stream

	printf_P(PSTR("\n\rAVR488 interface started (V%s " __DATE__ " " __TIME__ ")\n\r"), VERSION);

    // Read saved settings from the EEPROM
    if (eeprom_read_byte((uint8_t *)0x00) == VALID_EEPROM_CODE)
	{
        partnerAddress = eeprom_read_byte((uint8_t *)0x02);
        eot_char = eeprom_read_byte((uint8_t *)0x03);
		eot_enable = eeprom_read_byte((uint8_t *)0x04);
        set_eos_string(eeprom_read_byte((uint8_t *)0x05));
        eoiUse = eeprom_read_byte((uint8_t *)0x06);
        autoread = eeprom_read_byte((uint8_t *)0x07);
		debug = eeprom_read_byte((uint8_t *)0x10);
		localecho = eeprom_read_byte((uint8_t *)0x11);
    }
    else // default values if EEPROM entries aren't valid
	{
        eeprom_update_byte((uint8_t *)0x00, VALID_EEPROM_CODE);
        eeprom_update_byte((uint8_t *)0x02, 1); // partnerAddress
        eeprom_update_byte((uint8_t *)0x03, 13); // eot_char
        eeprom_update_byte((uint8_t *)0x04, 0); // eot_enable
        eeprom_update_byte((uint8_t *)0x05, 3); // eos_code
        eeprom_update_byte((uint8_t *)0x06, 1); // eoiUse
        eeprom_update_byte((uint8_t *)0x07, 1); // autoread
        eeprom_update_byte((uint8_t *)0x10, 1); // debug
        eeprom_update_byte((uint8_t *)0x11, 1); // echo
    }
	_wdt_reset();
	_delay_ms(200);

	// Start all the GPIB related stuff
	prep_gpib_pins(); // Initialize the GPIB Bus
    gpib_controller_assign();

	_wdt_reset();
	_delay_ms(200);

	// Decode the cause of the CPU restart
	if (debug == 1)
	{
		if ( mcusr & 1<<WDRF)
			printf_P(PSTR("Restart by watchdog.\n\r"));
		if ( mcusr & 1<<BORF)
			printf_P(PSTR("Restart by brownout detector.\n\r"));
		if ( mcusr & 1<<EXTRF)
			printf_P(PSTR("Restart by external cause.\n\r"));
		if ( mcusr & 1<<PORF)
			printf_P(PSTR("Restart by power-on.\n\r"));
	}

	// Main execution loop - never ends
	while(1)
	{
		uart_gets(buf, (sizeof buf)-1, localecho); // Get a null terminated string from the USB interface
		if (localecho)
		{
			uart_putc('\n'); // send a carriage return and line feed
			uart_putc('\r');
		}

		if(buf[0] == '+' && buf[1] == '+') // Controller commands start with a ++
		{
			// ++addr N
			if(IS_CMD("addr"))
			{
				if (*(buf+6) == 0)
				        printf_P(PSTR("%i\n\r"), partnerAddress);
				else if (*(buf+6) == ' ')
				{
				    addrTemp = atoi((buf+7));
					if (addrTemp<1 || addrTemp>30)
						printf_P(PSTR("Invalid instrument address\n\r"));
					else
					{
						partnerAddress = addrTemp;
						printf_P(PSTR("OK\n\r"));
					}
				}
			}

			// ++auto {0|1}
			else if(IS_CMD("auto"))
			{
				if (*(buf+6) == 0) {
					printf_P(PSTR("%i\n\r"), autoread);
				}
				else if (*(buf+6) == ' ')
				{
					autoread = atoi((buf+7));
					if ((autoread != 0) && (autoread != 1))
						autoread = 1; // If non-bool sent, set to enable
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++clr
			else if(IS_CMD("clr"))
			{
				writeError = addressTarget(partnerAddress);
				cmd_buf[0] = CMD_SDC;
				writeError = writeError || gpib_cmd(cmd_buf,1);
				if (writeError)
					printf_P(PSTR("ERROR\n\r"));
				else
					printf_P(PSTR("OK\n\r"));
			}

			// ++debug {0|1}
			else if(IS_CMD("debug"))
			{
				if (*(buf+7) == 0)
					printf_P(PSTR("%i\n\r"), debug);
				else if (*(buf+7) == ' ')
				{
					debug = atoi((buf+8));
					if ((debug != 0) && (debug != 1))
						debug = 0; // If non-bool sent, set to disabled
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++echo {0|1}
			else if(IS_CMD("echo"))
			{
				if (*(buf+6) == 0)
					printf_P(PSTR("%i\n\r"), localecho);
				else if (*(buf+6) == ' ')
				{
					localecho = atoi((buf+7));
					if ((localecho != 0) && (localecho != 1))
						localecho = 0; // If non-bool sent, set to disabled
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++eoi {0|1}
			else if(IS_CMD("eoi"))
			{
				if (*(buf+5) == 0)
					printf_P(PSTR("%i\n\r"), eoiUse);
				else if (*(buf+5) == ' ')
				{
					eoiUse = atoi((buf+6));
					if ((eoiUse != 0) && (eoiUse != 1))
					eoiUse = 0; // If non-bool sent, set to disabled
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++eos {0|1|2|3}
			else if(IS_CMD("eos"))
			{
				if (*(buf+5) == 0) {
					printf_P(PSTR("%i\n\r"), eos_code);
				}
				else if (*(buf+5) == ' ')
				{
					set_eos_string(atoi((buf+6)));
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++eot_char N
			else if(IS_CMD("eot_char"))
			{
				if (*(buf+10) == 0)
					printf_P(PSTR("%i\n\r"), eot_char);
				else if (*(buf+10) == ' ')
				{
					eot_char = atoi((buf+11));
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++eot_enable {0|1}
			else if(IS_CMD("eot_enable"))
			{
				if (*(buf+12) == 0)
				printf_P(PSTR("%i\n\r"), eot_enable);
				else if (*(buf+12) == ' ')
				{
					eot_enable = atoi((buf+13));
					if ((eot_enable != 0) && (eot_enable != 1))
						eot_enable = 0; // If non-bool sent, set to disabled
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++help
			else if(IS_CMD("help"))
			{
				print_help();
			}

			// ++ifc
			else if(IS_CMD("ifc"))
			{
				ASSERT(IFC); // Assert interface clear.
				_delay_us(150);
				FLOAT(IFC); // Finishing clearing interface
				printf_P(PSTR("OK\n\r"));
			}

			// ++llo
			else if(IS_CMD("llo"))
			{
				writeError = addressTarget(partnerAddress);
				cmd_buf[0] = CMD_LLO;
				writeError = writeError || gpib_cmd(cmd_buf,1);
				if (writeError)
					printf_P(PSTR("ERROR\n\r"));
				else
					printf_P(PSTR("OK\n\r"));
			}

			// ++loc
			else if(IS_CMD("loc"))
			{
				writeError = addressTarget(partnerAddress);
				cmd_buf[0] = CMD_GTL;
				writeError = writeError || gpib_cmd(cmd_buf,1);
				if (writeError)
					printf_P(PSTR("ERROR\n\r"));
				else
					printf_P(PSTR("OK\n\r"));
			}

			// ++mode
			else if(IS_CMD("mode"))
			{
				if (*(buf+6) == 0)
					printf_P(PSTR("1\n\r"));
				else if (*(buf+6) == ' ')
				{
					if (atoi(buf+7) == 1)
						printf_P(PSTR("OK\n\r"));
					else
						printf_P(PSTR("ERROR\n\r"));
				}
			}

			// ++read_tmo_ms N
			else if(IS_CMD("read_tmo_ms"))
			{
				if (*(buf+13) == 0)
					printf_P(PSTR("%u\n\r"), timeout);
				else if (*(buf+13) == ' ')
				{
					timeout = atol((buf+14));
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++read {e}
			else if(IS_CMD("read"))
			{
				if (*(buf+6) == 0)
				    gpib_read(false, partnerAddress); // read until EOS condition
			    else if (*(buf+7) == 'e')
			        gpib_read(true, partnerAddress); // read until EOI flagged
			}

			// ++rst
			else if(IS_CMD("rst"))
			{
				while (true) {}; // block CPU to trigger WDT timeout and restart
			}

			// ++savecfg 1
			else if(IS_CMD("savecfg"))
			{
				if (*(buf+9) == 0)
					printf_P(PSTR("0\n\r"));
				else if (*(buf+9) == ' ')
				{
					if (atoi(buf+10) == 1)
					{
						eeprom_update_byte((uint8_t *)0x02, partnerAddress);
						eeprom_update_byte((uint8_t *)0x03, eot_char);
						eeprom_update_byte((uint8_t *)0x04, eot_enable);
						eeprom_update_byte((uint8_t *)0x05, eos_code);
						eeprom_update_byte((uint8_t *)0x06, eoiUse);
						eeprom_update_byte((uint8_t *)0x07, autoread);
						eeprom_update_byte((uint8_t *)0x10, debug);
						eeprom_update_byte((uint8_t *)0x11, localecho);
					}
					printf_P(PSTR("OK\n\r"));
				}
			}

			// ++spoll N
			else if(IS_CMD("spoll"))
			{
				if (*(buf+7) == 0)
					serial_poll(partnerAddress);
				else if (*(buf+7) == ' ')
					serial_poll(atoi((buf+8)));
			}

			// ++srq
			else if(IS_CMD("srq"))
			{
				printf_P(PSTR("%i\n\r"), srq_state());
			}

			// ++trg N
			else if(IS_CMD("trg"))
			{
				addrTemp = partnerAddress;
				if (*(buf+5) == ' ')
				addrTemp = atoi((buf+6));

				writeError = addressTarget(addrTemp);
				cmd_buf[0] = CMD_GET;
				writeError = writeError || gpib_cmd(cmd_buf, 1);
				if (writeError)
				printf_P(PSTR("ERROR\n\r"));
				else
				printf_P(PSTR("OK\n\r"));
			}

			// ++ver
			else if(IS_CMD("ver"))
			{
				printf_P(PSTR("%s\n\r"), VERSION);
			}

			// ++xdiag
			else if(IS_CMD("xdiag"))
			{
				printf_P(PSTR("     76543210\n\r"));
				printf_P(PSTR("PINB=%8s\n\r"), utoa(PINB,buffer,2));
				printf_P(PSTR("PINC=%8s\n\r"), utoa(PINC,buffer,2));
				printf_P(PSTR("PIND=%8s\n\r"), utoa(PIND,buffer,2));
			}

			else
			{
				if (debug == 1)
					printf_P(PSTR("Unrecognized command\n\r"));
				else
					printf_P(PSTR("ERROR\n\r"));
			}
		}
		else
		{
		    // Not an internal command, send to GPIB bus
			// Command all talkers and listeners to stop
			// and tell target to listen and send out command to the bus

			writeError = addressTarget(partnerAddress);
			if (eos_code != 3) // If have an EOS code, need to append termination byte(s)
				strcat(buf, eos_string);
            writeError = writeError || gpib_write(buf, 0); // send to the instrument

			// If were in auto mode and command contains a question mark this is a query to the instrument, so automatically read the response
			if (autoread)
			{
				if ((strchr(buf, '?') != NULL) && !(writeError))
					writeError = gpib_read(eoiUse, partnerAddress);
				else
				{
					if (localecho)
					{
						if (writeError)
							printf_P(PSTR("ERROR\n\r"));
						else
							printf_P(PSTR("OK\n\r"));
					}
				}
			}
		}
    }
}
