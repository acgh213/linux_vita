/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Clock ids for the PS Vita pervasive clock-gate block (0xE3102000).
 */
#ifndef _DT_BINDINGS_CLOCK_VITA_PERVASIVE_H
#define _DT_BINDINGS_CLOCK_VITA_PERVASIVE_H

#define VITA_PCLK_USB0		0
#define VITA_PCLK_USB1		1
#define VITA_PCLK_USB2		2

/*
 * These ids are append-only.  Keep the USB ids above stable: they are used
 * by the existing Vita DTBs and their host-mode clock semantics.
 */
#define VITA_PCLK_DSI0		3
#define VITA_PCLK_DSI1		4
#define VITA_PCLK_GPIO		5
#define VITA_PCLK_SPI0		6
#define VITA_PCLK_UART0		7
#define VITA_PCLK_MSIF		8

#define VITA_PCLK_NR		9

#endif
