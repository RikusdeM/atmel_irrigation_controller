/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000

// ************************************************************************

#define UART_NONE			0
#define UARTC1				1
#define UARTE0				2

#define DEBUG_UART			UARTC1
#define MODEM_UART			UARTE0

#if DEBUG_UART == UARTC1 || MODEM_UART == UARTC1
#define ENABLE_UARTC1		1
#else
#define ENABLE_UARTC1		0
#endif

#if DEBUG_UART == UARTE0 || MODEM_UART == UARTE0
#define ENABLE_UARTE0		1
#else
#define ENABLE_UARTC1		0
#endif

// ************************************************************************

#define GPIO_COUNT4			IOPORT_CREATE_PIN(PORTA,2)
#define GPIO_X1				IOPORT_CREATE_PIN(PORTA,3)
#define GPIO_LED2           IOPORT_CREATE_PIN(PORTA,5)
#define GPIO_X3				IOPORT_CREATE_PIN(PORTA,6)

#define GPIO_SL_WDOG        IOPORT_CREATE_PIN(PORTB,1)
#define GPIO_COUNT3			IOPORT_CREATE_PIN(PORTB,2)
#define GPIO_LED1           IOPORT_CREATE_PIN(PORTB,3)

#define GPIO_COUNT1			IOPORT_CREATE_PIN(PORTC,2)
#if ENABLE_UARTC1 == 1
#define GPIO_USARTC1_RXD1   IOPORT_CREATE_PIN(PORTC,6)
#define GPIO_USARTC1_TXD1   IOPORT_CREATE_PIN(PORTC,7)
#else
#define GPIO_COUNT5			IOPORT_CREATE_PIN(PORTC,6)
#define GPIO_COUNT6			IOPORT_CREATE_PIN(PORTC,7)
#endif

#define GPIO_EXT_OP1        IOPORT_CREATE_PIN(PORTD,0)
#define GPIO_EXT_OP2        IOPORT_CREATE_PIN(PORTD,1)
#define GPIO_COUNT2			IOPORT_CREATE_PIN(PORTD,2)
#define GPIO_EXT_OP3        IOPORT_CREATE_PIN(PORTD,3)

#define GPIO_POWER			GPIO_EXT_OP3


#if ENABLE_UARTE0 == 1
#define GPIO_USARTE0_RXD0   IOPORT_CREATE_PIN(PORTE,2)
#define GPIO_USARTE0_TXD0   IOPORT_CREATE_PIN(PORTE,3)
#endif


#define TWI_IESM            (&TWIC)

#endif // USER_BOARD_H
