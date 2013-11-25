/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
    ioport_configure_pin(GPIO_LED1, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin(GPIO_LED2, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);

    ioport_configure_pin(GPIO_USARTE0_TXD0, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin(GPIO_USARTE0_RXD0, IOPORT_DIR_INPUT);

#if ENABLE_UARTE0 == 1
	ioport_configure_pin(GPIO_USARTE0_TXD0, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(GPIO_USARTE0_RXD0, IOPORT_DIR_INPUT);
#endif

#if ENABLE_UARTC1 == 1
	ioport_configure_pin(GPIO_USARTC1_TXD1, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(GPIO_USARTC1_RXD1, IOPORT_DIR_INPUT);
#endif

    ioport_configure_pin(GPIO_SL_WDOG, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);

    ioport_configure_pin(GPIO_X3, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);


    ioport_configure_pin(GPIO_EXT_OP1, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
    ioport_configure_pin(GPIO_EXT_OP2, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
    ioport_configure_pin(GPIO_EXT_OP3, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);

	ioport_configure_pin(GPIO_COUNT1, IOPORT_DIR_INPUT);
	ioport_configure_pin(GPIO_COUNT2, IOPORT_DIR_INPUT);
	ioport_configure_pin(GPIO_COUNT3, IOPORT_DIR_INPUT);
	ioport_configure_pin(GPIO_COUNT4, IOPORT_DIR_INPUT);
}
