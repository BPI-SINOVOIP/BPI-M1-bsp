 /*sunxi_lirc_new.h
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *Created on: 22 janv. 2016
 *Author: Damien Pageot
 */
#ifndef SUNXI_LIRC_NEW_H_
#define SUNXI_LIRC_NEW_H_
/* utility */
#define GENMASK(h, l) \
       (((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))

#define LIRC
#define LIRC_DRIVER_NAME "sunxi_lirc_new"
#define IR_RAW_BUF_SIZE 512

/* Registers */

/* base ir register */
#define IR_BASE		(0xf1c21800) // to be delete if device is implemented in devices.c see comment in .c
/* IRQ */
#define IR_IRQNO	(SW_INT_IRQNO_IR0)
/* IR Control */
#define SUNXI_IR_CTL_REG      0x00
/* Global Enable */
#define REG_CTL_GEN			BIT(0)
/* RX block enable */
#define REG_CTL_RXEN			BIT(1)
/* CIR mode */
#define REG_CTL_MD			(BIT(4) | BIT(5))

/* Rx Config */
#define SUNXI_IR_RXCTL_REG    0x10
/* Pulse Polarity Invert flag */
#define REG_RXCTL_RPPI			BIT(2)

/* Rx Data */
#define SUNXI_IR_RXFIFO_REG   0x20

/* Rx Interrupt Enable */
#define SUNXI_IR_RXINT_REG    0x2C
/* Rx FIFO Overflow */
#define REG_RXINT_ROI_EN		BIT(0)
/* Rx Packet End */
#define REG_RXINT_RPEI_EN		BIT(1)
/* Rx FIFO Data Available */
#define REG_RXINT_RAI_EN		BIT(4)

/* Rx FIFO available byte level */
#define REG_RXINT_RAL(val)    (((val) << 8) & (GENMASK(11, 8)))

/* Rx Interrupt Status */
#define SUNXI_IR_RXSTA_REG    0x30
/* RX FIFO Get Available Counter */
#define REG_RXSTA_GET_AC(val) (((val) >> 8) & (GENMASK(5, 0)))
/* Clear all interrupt status value */
#define REG_RXSTA_CLEARALL    0xff

/* IR Sample Config */
#define SUNXI_IR_CIR_REG      0x34
/* CIR_REG register noise threshold */
#define REG_CIR_NTHR(val)    (((val) << 2) & (GENMASK(7, 2)))
/* CIR_REG register idle threshold */
#define REG_CIR_ITHR(val)    (((val) << 8) & (GENMASK(15, 8)))

/* Hardware supported fifo size */
#define SUNXI_IR_FIFO_SIZE    16 /*This size is correct but in datasheet
Allwinner say that the FIFO is 64 byte long but on registers only 16 byte are accessible*/
/* How many messages in FIFO trigger IRQ */
#define TRIGGER_LEVEL         8 // half of the total size
/* Required frequency for IR0 or IR1 clock in CIR mode */
#define SUNXI_IR_BASE_CLK     8000000
/* Frequency after IR internal divider  */
#define SUNXI_IR_CLK          (SUNXI_IR_BASE_CLK / 64)
/* Sample period in us */
#define SUNXI_IR_SAMPLE       (1000000000ul / SUNXI_IR_CLK) //sample rate is 8us
/* Noise threshold in samples  */
#define SUNXI_IR_RXNOISE      1
/* Idle Threshold in samples *//* Idle Threshold = (20+1)*128*sample = ~21ms */
#define SUNXI_IR_RXIDLE       20
/* Time after which device stops sending data in ms */
#define SUNXI_IR_TIMEOUT      120

#define dprintk(fmt, args...)           \
do {                                    \
if (debug)                              \
printk(KERN_DEBUG LIRC_DRIVER_NAME ": " \
fmt, ## args);                          \
} while (0)

#endif /* SUNXI_LIRC_NEW_H_ */
