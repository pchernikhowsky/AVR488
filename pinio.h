/*
 * pinio.h
 *
 * Created: 30-Dec-2021 3:54:58 PM
 *  
 */


#ifndef PINIO_H_
#define PINIO_H_

// ATmega168/328 -> GPIB pin mapping (using the MCP2221A)
//
//                       +-----------\_/-----------+
//        /RESET        -|1  PC6             PC5 28|-        GPIB 11: ATN
// MCP2221 6: TX        -|2  PD0             PC4 27|-        GPIB 10: SRQ
// MCP2221 5: RX        -|3  PD1             PC3 26|-        GPIB  9: IFC
//    STATUS LED        -|4  PD2             PC2 25|-        GPIB  8: NDAC
//  GPIB 17: REN        -|5  PD3             PC1 24|-        GPIB  7: NRFD
//   GPIB 5: EOI        -|6  PD4             PC0 23|-        GPIB  6: DAV
//                  Vcc -|7                      22|- GND
//                  GND -|8                      21|- AREF
//  GPIB 15: DIO7       -|9  PB6                 20|- AVcc
//  GPIB 16: DIO8       -|10 PB7             PB5 19|-        GPIB 14: DIO6
//     SN75161 DC       -|11 PD5             PB4 18|-        GPIB 13: DIO5
// SN75160/161 TE       -|12 PD6             PB3 17|-        GPIB  4: DIO4
//     SN75160 PE       -|13 PD7             PB2 16|-        GPIB  3: DIO3
//   GPIB 1: DIO1       -|14 PB0             PB1 15|-        GPIB  2: DIO2
//                       +-------------------------+

// AVR port to GPIB pin definitions
#define DAV  PINC0
#define NRFD PINC1
#define NDAC PINC2
#define IFC  PINC3
#define SRQ  PINC4
#define ATN  PINC5

// Use the high bit to indicate PORTD pins
#define REN (PIND3 | 0x80)
#define EOI (PIND4 | 0x80)
#define DC  (PIND5 | 0x80)
#define TE  (PIND6 | 0x80)
#define PE  (PIND7 | 0x80)
#define LED_STATUS (PIND2 | 0x80) // continuously toggled by _wdt_reset() to indicate life

// Defines for pin control using GPIB negative logic
#define ASSERT output_low
#define DEASSERT output_high
#define FLOAT output_float

// Prototypes for pin control functions
extern void output_float(uint8_t bit);
extern void output_high(uint8_t bit);
extern void output_low(uint8_t bit);
extern void toggle_output(uint8_t bit);
extern uint8_t input(uint8_t bit);

#endif /* PINIO_H_ */