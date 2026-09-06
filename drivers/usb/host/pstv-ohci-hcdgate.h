/* SPDX-License-Identifier: GPL-2.0 */
#ifndef PSTV_OHCI_HCDGATE_H
#define PSTV_OHCI_HCDGATE_H

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/usb.h>

struct usb_hcd;

int gate_hcd_up(struct platform_device *pdev, void __iomem *regs,
		unsigned int irq);
void gate_hcd_down(void);
bool gate_hcd_active(void);

#endif
