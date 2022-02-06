/*
 * pinio.c
 *
 * Pin control functions and assignments
 *
 * Created: 29-Dec-2021 10:40:59 AM
 *  Author: Paul Chernikhowsky
 */

#include <avr/io.h>
#include "pinio.h"

void output_float(uint8_t bit)
{
	// if high bit is set, send to port D, otherwise it's port C
	if (bit&0x80)
	{
		bit &= 0x7f;	// strip the high bit
		DDRD &= ~(1<<bit); // set pin as input
		PORTD |= 1<<bit; // enable pullup
	}
	else
	{
		DDRC &= ~(1<<bit); // set pin as input
		PORTC |= 1<<bit; // enable pullup
	}
}

void output_high(uint8_t bit)
{
	// if high bit is set, send to port D, otherwise it's port C
	if (bit&0x80)
	{
		bit &= 0x7f;	// strip the high bit
		DDRD |= (1<<bit);
		PORTD |= (1<<bit);
	}
	else
	{
		DDRC |= 1<<bit;
		PORTC |= 1<<bit;
	}
}

void output_low(uint8_t bit)
{
	// if high bit is set, send to port D, otherwise it's port C
	if (bit&0x80)
	{
		bit &= 0x7f;	// strip the high bit
		DDRD |= (1<<bit);
		PORTD &= ~(1<<bit);
	}
	else
	{
		DDRC |= 1<<bit;
		PORTC &= ~(1<<bit);
	}
}

void toggle_output(uint8_t bit)
{
	// if high bit is set, send to port D, otherwise it's port C
	if (bit&0x80)
	{
		bit &= 0x7f;	// strip the high bit
		PIND = (1<<bit);
	}
	else
	{
		PINC = 1<<bit;
	}
}

uint8_t input(uint8_t bit)
{
	// if high bit is set, input from port D, otherwise it's port C
	if (bit&0x80)
	{
		bit &= 0x7f;	// strip the high bit
		return (PIND & (1<<bit))?1:0;
	}
	else
	{
		return (PINC & (1<<bit))?1:0;
	}
}

