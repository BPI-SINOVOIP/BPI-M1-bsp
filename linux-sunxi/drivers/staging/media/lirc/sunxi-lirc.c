/*
 *
 * sunxi_lirc.c
 *
 * sunxi_lirc - Device driver that uses Allwinner A1X or A20 IR module in CIR mode
 *              for LIRC. Tested on a Cubietruck with Allwinner A20
 *              a lot of code from the sunxi-ir module,
 *              so I would like say thanks to the authors.
 *        		Difference to sunxi-ir is that no verification of IR code
 *        		against NEC protocol is made, whatsoever
 *        		but just passed on to lirc buffer to let lirc do any decoding
 *
 * Copyright (C) 2014 Matthias Hoelling <mhoel....@gmail.nospam.com>,
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <mach/clock.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>

#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <plat/sys_config.h>

#include <linux/clk.h>

#define LIRC_DRIVER_NAME "sunxi_lirc"
#define RBUF_LEN 256

static struct platform_device *lirc_sunxi_dev;

static struct clk *apb_ir_clk;
static struct clk *ir_clk;
static u32 ir_gpio_hdle;

#define SYS_CLK_CFG_EN

#define SYS_GPIO_CFG_EN
/* #define DEBUG_IR */
#define PRINT_SUSPEND_INFO

#define dprintk(fmt, args...)                                        \
do {                                                        \
if (debug)                                        \
printk(KERN_DEBUG LIRC_DRIVER_NAME ": "        \
fmt, ## args);                        \
} while (0)

/* Registers */
#define IR_REG(x)	(x)
#define IR0_BASE	(0xf1c21800)
#define IR1_BASE	(0xf1c21c00)
#define IR_BASE		IR0_BASE
#define IR_IRQNO	(SW_INT_IRQNO_IR0)

/* CCM register */
#define CCM_BASE	0xf1c20000
/* PIO register */
#define PI_BASE		0xf1c20800

#define IR_CTRL_REG	IR_REG(0x00) /* IR Control */
#define IR_RXCFG_REG	IR_REG(0x10) /* Rx Config */
#define IR_RXDAT_REG	IR_REG(0x20) /* Rx Data */
#define IR_RXINTE_REG	IR_REG(0x2c) /* Rx Interrupt Enable */
#define IR_RXINTS_REG	IR_REG(0x30) /* Rx Interrupt Status */
#define IR_SPLCFG_REG	IR_REG(0x34) /* IR Sample Config */


/* Bit Definition of IR_RXINTS_REG Register */
#define IR_RXINTS_RXOF	(0x1 << 0)	/* Rx FIFO Overflow */
#define IR_RXINTS_RXPE	(0x1 << 1)	/* Rx Packet End */
#define IR_RXINTS_RXDA	(0x1 << 4)	/* Rx FIFO Data Available */

/* Helper */
#define PULSE_BIT_SHIFTER	(17)  /* from 0x80 to PULSE_BIT */
#define SAMPLES_TO_US(us)	(( ((unsigned long) us) * 1000000UL)/46875UL)

#ifdef CONFIG_ARCH_SUN5I
#define IR_FIFO_SIZE	(64)	/* 64Bytes */
#else
#define IR_FIFO_SIZE	(16)	/* 16Bytes */
#endif
/* Frequency of Sample Clock = 46875.Hz, Cycle is 21.3us */

#define IR_RXFILT_VAL	(8)	/* Filter Threshold = 8*21.3 = ~128us	< 200us */
#define IR_RXIDLE_VAL	(5)	/* Idle Threshold = (5+1)*128*21.3 = ~16.4ms > 9ms */


#define	IR_RAW_BUF_SIZE	128
#define DRV_VERSION	"1.00"



struct ir_raw_buffer {
	unsigned int dcnt;	/*Packet Count*/
	unsigned char buf[IR_RAW_BUF_SIZE];
};

static struct lirc_buffer rbuf;

DEFINE_SPINLOCK(sunxi_lirc_spinlock);



static struct ir_raw_buffer ir_rawbuf;

static int debug=0;



static inline void ir_reset_rawbuffer(void)
{
	ir_rawbuf.dcnt = 0;
}

static inline void ir_write_rawbuffer(unsigned char data)
{
	if (ir_rawbuf.dcnt < IR_RAW_BUF_SIZE)
		ir_rawbuf.buf[ir_rawbuf.dcnt++] = data;
	else
		printk("ir_write_rawbuffer: IR Rx buffer full\n");
}

static inline unsigned char ir_read_rawbuffer(void)
{
	unsigned char data = 0x00;

	if (ir_rawbuf.dcnt > 0)
		data = ir_rawbuf.buf[--ir_rawbuf.dcnt];

	return data;
}

static inline int ir_rawbuffer_empty(void)
{
	return (ir_rawbuf.dcnt == 0);
}

static inline int ir_rawbuffer_full(void)
{
	return (ir_rawbuf.dcnt >= IR_RAW_BUF_SIZE);
}

static void ir_clk_cfg(void)
{
#ifdef SYS_CLK_CFG_EN
	unsigned long rate = 3000000; /* 6 MHz */
#else
	unsigned long tmp = 0;
#endif

#ifdef SYS_CLK_CFG_EN
	apb_ir_clk = clk_get(NULL, "apb_ir0");
	if (!apb_ir_clk) {
		printk("try to get apb_ir0 clock failed\n");
		return;
	}

	ir_clk = clk_get(NULL, "ir0");
	if (!ir_clk) {
		printk("try to get ir0 clock failed\n");
		return;
	}
	dprintk("trying to set clock via SYS_CLK_CFG_EN, when no error follows -> succeeded\n");
	if (clk_set_rate(ir_clk, rate))
		printk("set ir0 clock freq to 6M failed\n");

	if (clk_enable(apb_ir_clk))
		printk("try to enable apb_ir_clk failed\n");

	if (clk_enable(ir_clk))
		printk("try to enable apb_ir_clk failed\n");

#else
	dprintk("setting clock via register manipulation\n");
	/* Enable APB Clock for IR */
	/* copied from sunxi-ir.c, but not sure if this will work, registers do not seem conform with data sheet */
	tmp = readl(CCM_BASE + 0x10);
	tmp |= 0x1 << 10;  /* IR */
	writel(tmp, CCM_BASE + 0x10);

	/* config Special Clock for IR (24/8=6MHz) */
	tmp = readl(CCM_BASE + 0x34);
	tmp &= ~(0x3 << 8);
	tmp |= (0x1 << 8);	/* Select 24MHz */
	tmp |= (0x1 << 7);	/* Open Clock */
	tmp &= ~(0x3f << 0);
	tmp |= (7 << 0);	/* Divisor = 8 */
	writel(tmp, CCM_BASE + 0x34);
#endif

	return;
}

static void ir_clk_uncfg(void)
{
#ifdef SYS_CLK_CFG_EN
	clk_put(apb_ir_clk);
	clk_put(ir_clk);
#endif

	return;
}
static void ir_sys_cfg(void)
{
#ifdef SYS_GPIO_CFG_EN
	ir_gpio_hdle = gpio_request_ex("ir_para", "ir0_rx");
	if (0 == ir_gpio_hdle)
		printk("try to request ir_para gpio failed\n");
#else
	/* config IO: PIOB4 to IR_Rx */
	tmp = readl(PI_BASE + 0x24); /* PIOB_CFG0_REG */
	tmp &= ~(0xf << 16);
	tmp |= (0x2 << 16);
	writel(tmp, PI_BASE + 0x24);
#endif

	ir_clk_cfg();

	return;
}

static void ir_sys_uncfg(void)
{
#ifdef SYS_GPIO_CFG_EN
	gpio_release(ir_gpio_hdle, 2);
#endif
	ir_clk_uncfg();

	return;
}

static void ir_reg_cfg(void)
{
	unsigned long tmp = 0;
	/* Enable CIR Mode */
	tmp = 0x3 << 4;
	writel(tmp, IR_BASE + IR_CTRL_REG);

	/* Config IR Sample Register */
	tmp = 0x0 << 0; /* Fsample = 3MHz/64 =46875Hz (21.3us) */


	tmp |= (IR_RXFILT_VAL & 0x3f) << 2; /* Set Filter Threshold */
	tmp |= (IR_RXIDLE_VAL & 0xff) << 8; /* Set Idle Threshold */
	writel(tmp, IR_BASE + IR_SPLCFG_REG);

	/* Invert Input Signal */
	writel(0x1 << 2, IR_BASE + IR_RXCFG_REG);


	/* Clear All Rx Interrupt Status */
	writel(0xff, IR_BASE + IR_RXINTS_REG);

	/* Set Rx Interrupt Enable */
	tmp = (0x1 << 4) | 0x3;
#ifdef CONFIG_ARCH_SUN5I
	tmp |= ((IR_FIFO_SIZE >> 2) - 1) << 8; /* Rx FIFO Threshold = FIFOsz/4 */
#else
	tmp |= ((IR_FIFO_SIZE >> 1) - 1) << 8; /* Rx FIFO Threshold = FIFOsz/2 */
#endif
	writel(tmp, IR_BASE + IR_RXINTE_REG);

	/* Enable IR Module */
	tmp = readl(IR_BASE + IR_CTRL_REG);
	tmp |= 0x3;
	writel(tmp, IR_BASE + IR_CTRL_REG);

	return;
}



static void ir_setup(void)
{
	dprintk("ir_setup: ir setup start!!\n");

	ir_reset_rawbuffer();
	ir_sys_cfg();
	ir_reg_cfg();

	dprintk("ir_setup: ir setup end!!\n");

	return;
}

static inline unsigned char ir_get_data(void)
{
	return (unsigned char)(readl(IR_BASE + IR_RXDAT_REG));
}

static inline unsigned long ir_get_intsta(void)
{
	return readl(IR_BASE + IR_RXINTS_REG);
}

static inline void ir_clr_intsta(unsigned long bitmap)
{
	unsigned long tmp = readl(IR_BASE + IR_RXINTS_REG);

	tmp &= ~0xff;
	tmp |= bitmap&0xff;
	writel(tmp, IR_BASE + IR_RXINTS_REG);
}



void ir_packet_handler(unsigned char *buf, unsigned int dcnt)
{
	unsigned int i;
	unsigned  int lirc_val;
	dprintk("Buffer length: %d",dcnt);
	for(i=0;i<dcnt;i++) {
		lirc_val= ((unsigned int) (buf[i] & 0x80) << PULSE_BIT_SHIFTER) | (SAMPLES_TO_US(buf[i] &0x7f));
		while((buf[i] & 0x80) == (buf[i+1] & 0x80)) {
			lirc_val += SAMPLES_TO_US(buf[++i]&0x7f);
		}
		/* statistically pulses are one sample period (?) too long, spaces too short */
		/* would make sense because of bandpass latency, but not sure... */
		lirc_val += (buf[i] & 0x80) ? (-SAMPLES_TO_US(1)) : SAMPLES_TO_US(1);
		dprintk("rawbuf: %x, value: %x, level:%d for %d us\n",buf[i],lirc_val,(buf[i]&0x80) ? 1 : 0,lirc_val & PULSE_MASK) ;
		/* to do, write to lirc buffer */
	    if (lirc_buffer_full(&rbuf)) {
	            /* no new signals will be accepted */
	            dprintk("Buffer overrun\n");
	            return;
	    }
	    lirc_buffer_write(&rbuf,(unsigned char*)&lirc_val);
	}


	return;
}



static irqreturn_t ir_irq_service(int irqno, void *dev_id)
{
	unsigned int dcnt;
	unsigned int i = 0;
	unsigned long intsta;

	intsta = ir_get_intsta();

	ir_clr_intsta(intsta);

	/* Read Data Every Time Enter this Routine*/
#ifdef CONFIG_ARCH_SUN5I
	dcnt = (unsigned int) (ir_get_intsta() >> 8) & 0x3f;
#else
	dcnt = (unsigned int) (ir_get_intsta() >> 8) & 0x1f;
#endif

	/* Read FIFO */
	for (i = 0; i < dcnt; i++) {
		if (ir_rawbuffer_full()) {
			dprintk("ir_irq_service: raw buffer full\n");
			break;
		} else {
			ir_write_rawbuffer(ir_get_data());
		}
	}

	if (intsta & IR_RXINTS_RXPE) { /* Packet End */

		ir_packet_handler(ir_rawbuf.buf, ir_rawbuf.dcnt);
		dprintk("Buffer written\n");
		ir_rawbuf.dcnt = 0;

        wake_up_interruptible(&rbuf.wait_poll);


	}

	if (intsta & IR_RXINTS_RXOF) {/* FIFO Overflow */
		/* flush raw buffer */
		ir_reset_rawbuffer();
		dprintk("ir_irq_service: Rx FIFO Overflow!!\n");
	}

	return IRQ_HANDLED;
}



/* interpret lirc commands */
static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{

        switch (cmd) {
        case LIRC_GET_SEND_MODE:
            return -ENOIOCTLCMD;
            break;

            /* driver cannot send */
        case LIRC_SET_SEND_MODE:
            return -ENOSYS;
            break;

        case LIRC_GET_LENGTH:
            return -ENOSYS;
            break;

        case LIRC_SET_SEND_DUTY_CYCLE:
            return -ENOSYS;
             break;

        case LIRC_SET_SEND_CARRIER:
            return -ENOSYS;
            break;

        default:
            return lirc_dev_fop_ioctl(filep, cmd, arg);
        }
        return 0;
}


static int set_use_inc(void* data) {
	return 0;
}

static void set_use_dec(void* data) {

}

static const struct file_operations lirc_fops = {
        .owner                = THIS_MODULE,
        .unlocked_ioctl        = lirc_ioctl,
        .read                = lirc_dev_fop_read, // this and the rest is default
        .write                = lirc_dev_fop_write,
        .poll                = lirc_dev_fop_poll,
        .open                = lirc_dev_fop_open,
        .release        = lirc_dev_fop_close,
        .llseek                = no_llseek,
};

static struct lirc_driver driver = {
        .name                = LIRC_DRIVER_NAME,
        .minor                = -1,           // assing automatically
        .code_length        = 1,
        .sample_rate        = 0,
        .data                = NULL,
        .add_to_buf        = NULL,
        .rbuf                = &rbuf,
        .set_use_inc        = set_use_inc,
        .set_use_dec        = set_use_dec,
        .fops                = &lirc_fops,
        .dev                = NULL,
        .owner                = THIS_MODULE,
};

/* end of lirc device/driver stuff */

/* now comes THIS driver, above is lirc */
static struct platform_driver lirc_sunxi_driver = {
        .driver = {
                .name   = LIRC_DRIVER_NAME,
                .owner  = THIS_MODULE,
        },
};





static int __init ir_init(void)
{
	int result;
    /* Init read buffer. */
    result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
    if (result < 0)
            return -ENOMEM;

    result = platform_driver_register(&lirc_sunxi_driver);
    if (result) {
            printk(KERN_ERR LIRC_DRIVER_NAME
                   ": lirc register returned %d\n", result);
            goto exit_buffer_free;
    }

    lirc_sunxi_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
    if (!lirc_sunxi_dev) {
            result = -ENOMEM;
            goto exit_driver_unregister;
    }

    result = platform_device_add(lirc_sunxi_dev);
    if (result) {
    	platform_device_put(lirc_sunxi_dev);
    	goto exit_driver_unregister;
    }

	if (request_irq(IR_IRQNO, ir_irq_service, 0, "RemoteIR",
			(void*) 0)) {
		result = -EBUSY;
		goto exit_device_unregister;
	}

	ir_setup();

	printk("IR Initial OK\n");



    // 'driver' is the lirc driver
        driver.features = LIRC_CAN_SEND_PULSE | LIRC_CAN_REC_MODE2;

        driver.dev = &lirc_sunxi_dev->dev;  // link THIS platform device to lirc driver
        driver.minor = lirc_register_driver(&driver);

        if (driver.minor < 0) {
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": device registration failed with %d\n", result);

                result = -EIO;
                goto exit_free_irq;
        }

        printk(KERN_INFO LIRC_DRIVER_NAME ": driver registered!\n");

    return 0;


exit_free_irq:
	free_irq(IR_IRQNO, (void*) 0);

exit_device_unregister:
    platform_device_unregister(lirc_sunxi_dev);

exit_driver_unregister:
    platform_driver_unregister(&lirc_sunxi_driver);

exit_buffer_free:
    lirc_buffer_free(&rbuf);

	return result;
}

static void __exit ir_exit(void)
{

	free_irq(IR_IRQNO, (void*) 0);
	ir_sys_uncfg();
    platform_device_unregister(lirc_sunxi_dev);

    platform_driver_unregister(&lirc_sunxi_driver);

    lirc_buffer_free(&rbuf);
    lirc_unregister_driver(driver.minor);
    printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");

}

module_init(ir_init);
module_exit(ir_exit);

MODULE_DESCRIPTION("Remote IR driver");
MODULE_AUTHOR("Matthias Hoelling");
MODULE_LICENSE("GPL");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");

