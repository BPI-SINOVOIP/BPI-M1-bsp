/*
 * lirc_gpio.c
 *
 * lirc_gpio - Device driver that records pulse- and pause-lengths
 *              (space-lengths) (just like the lirc_serial driver does)
 *              between GPIO interrupt events.  Tested on a Cubieboard with Allwinner A10
 *        However, everything relies on the gpiolib.c module, so there is a good
 *        chance it will also run on other platforms.
 *              Lots of code has been taken from the lirc_rpi module, who in turn took a
 *        lot of code from the lirc_serial module,
 *              so I would like say thanks to the authors.
 *
 * Copyright (C) 2013 Matthias H��lling <mhoel....@gmail.nospam.com>,
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
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>
#include <linux/gpio.h>
#include <plat/sys_config.h>
#include <../drivers/gpio/gpio-sunxi.h>

#define LIRC_DRIVER_NAME "lirc_gpio"
/* this may have to be adapted for different platforms */
#define LIRC_GPIO_ID_STRING "A1X_GPIO"
#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 256

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

#define RX_OFFSET_GPIOCHIP gpio_in_pin - gpiochip->base
#define TX_OFFSET_GPIOCHIP gpio_out_pin - gpiochip->base

#define dprintk(fmt, args...)                                        \
do {                                                        \
if (debug)                                        \
printk(KERN_DEBUG LIRC_DRIVER_NAME ": "        \
fmt, ## args);                        \
} while (0)

/* module parameters */

/* set the default GPIO input pins, 3 channels */
static int gpio_in_pin = 0;
/* -1 = auto, 0 = active high, 1 = active low */
static int sense = -1;
static struct timeval lasttv = { 0, 0 };
static spinlock_t lock;

/* set the default GPIO output pin, 4 channels */
static int gpio_out_pin = 0;
/* enable debugging messages */
static int debug;
/* use softcarrier by default */
static int softcarrier = 1;
/* 0 = do not invert output, 1 = invert output */
static int invert = 0;

/* is the device open, so interrupt must be changed if pins are changed */
static int device_open = 0;

struct gpio_chip *gpiochip=NULL;
struct irq_chip *irqchip=NULL;
struct irq_data *irqdata=NULL;

/* forward declarations */
static long send_pulse(unsigned long length);
static void send_space(long length);
static void lirc_gpio_exit(void);

static struct platform_device *lirc_gpio_dev;
static struct lirc_buffer rbuf;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static unsigned long space_width;


/* stuff for TX pin */

static void safe_udelay(unsigned long usecs)
{
        while (usecs > MAX_UDELAY_US) {
                udelay(MAX_UDELAY_US);
                usecs -= MAX_UDELAY_US;
        }
        udelay(usecs);
}

static int init_timing_params(unsigned int new_duty_cycle,
                              unsigned int new_freq)
{
        /*
         * period, pulse/space width are kept with 8 binary places -
         * IE multiplied by 256.
         */
        if (256 * 1000000L / new_freq * new_duty_cycle / 100 <=
            LIRC_TRANSMITTER_LATENCY)
                return -EINVAL;
        if (256 * 1000000L / new_freq * (100 - new_duty_cycle) / 100 <=
            LIRC_TRANSMITTER_LATENCY)
                return -EINVAL;
        duty_cycle = new_duty_cycle;
        freq = new_freq;
        period = 256 * 1000000L / freq;
        pulse_width = period * duty_cycle / 100;
        space_width = period - pulse_width;
        /*printk(KERN_INFO "in init_timing_params, freq=%d pulse=%ld, "
           "space=%ld\n", freq, pulse_width, space_width); */
        return 0;
}

static long send_pulse_softcarrier(unsigned long length)
{
        int flag;
        unsigned long actual, target, d;

        length <<= 8;

        actual = 0; target = 0; flag = 0;
        while (actual < length) {
                if (flag) {
                        gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, invert);
                        target += space_width;
                } else {
                        gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, !invert);
                        target += pulse_width;
                }
                d = (target - actual -
                     LIRC_TRANSMITTER_LATENCY + 128) >> 8;
                /*
                 * Note - we've checked in ioctl that the pulse/space
                 * widths are big enough so that d is > 0
                 */
                udelay(d);
                actual += (d << 8) + LIRC_TRANSMITTER_LATENCY;
                flag = !flag;
        }
        return (actual-length) >> 8;
}

static long send_pulse(unsigned long length)
{
        if (length <= 0)
                return 0;

        if (softcarrier) {
                return send_pulse_softcarrier(length);
        } else {
                gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, !invert);
                safe_udelay(length);
                return 0;
        }
}


static void send_space(long length)
{
        gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, invert);
        if (length <= 0)
                return;
        safe_udelay(length);
}

/* end of TX stuff */



/* RX stuff: Handle interrupt and write vals to lirc buffer */

static void rbwrite(int l)
{
        if (lirc_buffer_full(&rbuf)) {
                /* no new signals will be accepted */
                dprintk("Buffer overrun\n");
                return;
        }
        lirc_buffer_write(&rbuf, (void *)&l);
}

static void frbwrite(int l)
{
        /* simple noise filter */
        static int pulse, space;
        static unsigned int ptr;

        if (ptr > 0 && (l & PULSE_BIT)) {
                pulse += l & PULSE_MASK;
                if (pulse > 250) {
                        rbwrite(space);
                        rbwrite(pulse | PULSE_BIT);
                        ptr = 0;
                        pulse = 0;
                }
                return;
        }
        if (!(l & PULSE_BIT)) {
                if (ptr == 0) {
                        if (l > 20000) {
                                space = l;
                                ptr++;
                                return;
                        }
                } else {
                        if (l > 20000) {
                                space += pulse;
                                if (space > PULSE_MASK)
                                        space = PULSE_MASK;
                                space += l;
                                if (space > PULSE_MASK)
                                        space = PULSE_MASK;
                                pulse = 0;
                                return;
                        }
                        rbwrite(space);
                        rbwrite(pulse | PULSE_BIT);
                        ptr = 0;
                        pulse = 0;
                }
        }
        rbwrite(l);
}

static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs)
{
        struct timeval tv;
        long deltv;
        int data;
        int signal;

        /* use the GPIO signal level */
        signal = gpiochip->get(gpiochip, RX_OFFSET_GPIOCHIP);

        /* unmask the irq */
        irqchip->irq_unmask(irqdata);

        if (sense != -1) {
                /* get current time */
                do_gettimeofday(&tv);

                /* calc time since last interrupt in microseconds */
                deltv = tv.tv_sec-lasttv.tv_sec;
                if (tv.tv_sec < lasttv.tv_sec ||
                    (tv.tv_sec == lasttv.tv_sec &&
                     tv.tv_usec < lasttv.tv_usec)) {
                printk(KERN_WARNING LIRC_DRIVER_NAME
                       ": AIEEEE: your clock just jumped backwards\n");
                printk(KERN_WARNING LIRC_DRIVER_NAME
                       ": %d %d %lx %lx %lx %lx\n", signal, sense,
                       tv.tv_sec, lasttv.tv_sec,
                       tv.tv_usec, lasttv.tv_usec);
                data = PULSE_MASK;
            } else if (deltv > 15) {
                data = PULSE_MASK; /* really long time */
                if (!(signal^sense)) {
                    /* sanity check */
                    printk(KERN_WARNING LIRC_DRIVER_NAME
                           ": AIEEEE: %d %d %lx %lx %lx %lx\n",
                           signal, sense, tv.tv_sec, lasttv.tv_sec,
                           tv.tv_usec, lasttv.tv_usec);
                    /*
                     * detecting pulse while this
                     * MUST be a space!
                     */
                    sense = sense ? 0 : 1;
                }
            } else {
                data = (int) (deltv*1000000 +
                              (tv.tv_usec - lasttv.tv_usec));
            }
                frbwrite(signal^sense ? data : (data|PULSE_BIT));
                lasttv = tv;
                wake_up_interruptible(&rbuf.wait_poll);
        }

        return IRQ_HANDLED;
}

/*  end of rx stuff */


/* setup pins, rx, tx, interrupts, active low/high.... */

static void set_sense(void)
{
    int i,nlow, nhigh;
    if (gpio_in_pin==0) {
        return;  // no rx
    }
    if (sense == -1) {
        /* wait 1/10 sec for the power supply */
        msleep(100);

        /*
         * probe 9 times every 0.04s, collect "votes" for
         * active high/low
         */
        nlow = 0;
        nhigh = 0;
        for (i = 0; i < 9; i++) {
            if (gpiochip->get(gpiochip, RX_OFFSET_GPIOCHIP))
                nlow++;
            else
                nhigh++;
            msleep(40);
        }
        sense = (nlow >= nhigh ? 1 : 0);
        printk(KERN_INFO LIRC_DRIVER_NAME
               ": auto-detected active %s receiver on GPIO pin %d\n",
               sense ? "low" : "high", gpio_in_pin);
    } else {
        printk(KERN_INFO LIRC_DRIVER_NAME
               ": manually/previously detected using active %s receiver on GPIO pin %d\n",
               sense ? "low" : "high", gpio_in_pin);
    }

}



static int setup_tx(int new_out_pin)
{
	int ret;
    user_gpio_set_t* pinstate;
    struct sunxi_gpio_chip* sgpio = container_of(gpiochip,struct sunxi_gpio_chip,chip);
    dprintk("addresses: gpiochip: %lx, sgpio: %lx",(unsigned long int) gpiochip,(unsigned long int) sgpio);
    if (gpio_out_pin==new_out_pin)
        return 0; //do not set up, pin not changed

    if (gpio_out_pin!=0) {   //we had tx pin setup.  Free it so others can use it!
        dprintk(": trying to free old out pin index %d: %s\n",gpio_out_pin,gpiochip->names[TX_OFFSET_GPIOCHIP]);
        gpio_free(gpio_out_pin);
    }
    gpio_out_pin=new_out_pin;
    if (gpio_out_pin==0) {
        return 0; // do not set up, TX disabled
    }
    dprintk(": trying to claim new out pin index %d: %s\n",gpio_out_pin,gpiochip->names[TX_OFFSET_GPIOCHIP]);
    ret = gpio_request(gpio_out_pin,LIRC_DRIVER_NAME "ir/out");
    if (ret){
        printk(KERN_ALERT LIRC_DRIVER_NAME
               ": cant claim gpio pin %d with code %d\n", gpio_out_pin,ret);
        ret = -ENODEV;
        goto exit_disable_tx;
    }
    gpiochip->direction_output(gpiochip, TX_OFFSET_GPIOCHIP, 1);
    gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, invert);
    pinstate=kzalloc(sizeof(user_gpio_set_t),GFP_KERNEL);

    //dprintk("pin:address %lx, pin_name: %s, pin handler: %d",(unsigned long int) &(sgpio->data[TX_OFFSET_GPIOCHIP]),
    // 		sgpio->data[TX_OFFSET_GPIOCHIP].pin_name,sgpio->data[TX_OFFSET_GPIOCHIP].gpio_handler);
    ret=gpio_get_one_pin_status(sgpio->data[TX_OFFSET_GPIOCHIP].gpio_handler, pinstate, sgpio->data[TX_OFFSET_GPIOCHIP].pin_name, true);
    if(pinstate && !ret) {
    	pr_info("Maximum load on '%s' is %d mA\n", gpiochip->names[TX_OFFSET_GPIOCHIP], 10+10*pinstate->drv_level);
    	kfree(pinstate);
    }
    else
    	printk(KERN_ALERT LIRC_DRIVER_NAME ": something might have gone wrong, return from pin status query: %d",ret);
    return 0;  // successfully set up

exit_disable_tx:
    // cannot claim new pin
    gpio_out_pin = 0; // disable tx
    return ret;
}

static int setup_rx(int new_in_pin)
{
    int ret,irq;
    if (gpio_in_pin==new_in_pin)
        return 0; //do not set up, pin not changed

    if (gpio_in_pin!=0) {   //we had rx pin setup.  Free it so others can use it!
        dprintk(": trying to free old in pin index %d: %s\n",gpio_in_pin,gpiochip->names[RX_OFFSET_GPIOCHIP]);
        gpio_free(gpio_in_pin);
        irqchip=NULL;
        irqdata=NULL;
    }
    gpio_in_pin=new_in_pin;
    if (gpio_in_pin==0) {
        return 0; // do not set up, RX disabled
    }
    dprintk(": trying to claim new in pin index %d: %s\n",gpio_in_pin,gpiochip->names[RX_OFFSET_GPIOCHIP]);
    ret = gpio_request(gpio_in_pin, LIRC_DRIVER_NAME " ir/in");
    if (ret) {
        printk(KERN_ALERT LIRC_DRIVER_NAME
               ": cant claim gpio pin %d with code %d\n", gpio_in_pin,ret);
        ret = -ENODEV;
        goto exit_disable_rx;
    }

    gpiochip->direction_input(gpiochip, RX_OFFSET_GPIOCHIP);
    /* try to setup interrupt data */
    irq = gpiochip->to_irq(gpiochip, RX_OFFSET_GPIOCHIP);
    dprintk("to_irq %d for pin %d\n", irq, gpio_in_pin);
    irqdata = irq_get_irq_data(irq);

    if (irqdata && irqdata->chip) {
        irqchip = irqdata->chip;
    } else {
        ret = -ENODEV;
        goto exit_gpio_free_in_pin;
    }

    set_sense();

    return 0; //successfully set up

exit_gpio_free_in_pin:
    // interrupt set up failed, so free pin
    gpio_free(gpio_in_pin);
exit_disable_rx:
    // could not claim new pin
    gpio_in_pin=0; // disable rx
    return ret;

}

/* end of pin setup */

/* get right gpio chip */

static int is_right_chip(struct gpio_chip *chip, const void *data)
{
        if (strcmp(data, chip->label) == 0)
                return 1;
        return 0;
}


static int set_gpiochip(void)
{
        gpiochip = gpiochip_find(LIRC_GPIO_ID_STRING, is_right_chip);

        if (!gpiochip)
                return -ENODEV;

        return 0;

}

/* end of find gpio chip */



/* lirc device stuff */

/* called when the character device is opened
   timing params initialized and interrupts activated */
static int set_use_inc(void *data)
{
        int result;
        unsigned long flags;

        init_timing_params(duty_cycle, freq);
        /* initialize pulse/space widths */

    //initialize timestamp, would not be needed if no RX
    do_gettimeofday(&lasttv);
    device_open++;

    if (gpio_in_pin!=0) { // entered if RX used
        /* try to set all interrupts to same handler, should work */
        result = request_irq(gpiochip->to_irq(gpiochip, RX_OFFSET_GPIOCHIP),
                             (irq_handler_t) irq_handler, 0,
                             LIRC_DRIVER_NAME, (void*) 0);

        switch (result) {
            case -EBUSY:
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": IRQ %d is busy\n",
                       gpiochip->to_irq(gpiochip, RX_OFFSET_GPIOCHIP));
                return -EBUSY;
            case -EINVAL:
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": Bad irq number or handler\n");
                return -EINVAL;
            default:
                dprintk("Interrupt %d obtained\n",
                        gpiochip->to_irq(gpiochip, RX_OFFSET_GPIOCHIP));
                break;
        };
        spin_lock_irqsave(&lock, flags);

        /* GPIO Pin Falling/Rising Edge Detect Enable */
        irqchip->irq_set_type(irqdata,
                              IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING);

        /* unmask the irq for active channel, only */
        irqchip->irq_unmask(irqdata);
        spin_unlock_irqrestore(&lock, flags);

    }

        return 0;
}

/* called when character device is closed */
static void set_use_dec(void *data)
{
        unsigned long flags;
    device_open--;

    if(!irqchip)
        return;

        /* GPIO Pin Falling/Rising Edge Detect Disable */
    spin_lock_irqsave(&lock, flags);
    irqchip->irq_set_type(irqdata, 0);
    irqchip->irq_mask(irqdata);
    spin_unlock_irqrestore(&lock, flags);

    free_irq(gpiochip->to_irq(gpiochip, RX_OFFSET_GPIOCHIP), (void *) 0);
    device_open--;
    dprintk("freed IRQ %d\n", gpiochip->to_irq(gpiochip, RX_OFFSET_GPIOCHIP));

}

/* lirc to tx */
static ssize_t lirc_write(struct file *file, const char *buf,
                          size_t n, loff_t *ppos)
{
        int i, count;
        unsigned long flags;

        long delta = 0;
        int *wbuf;
    if (!gpio_out_pin) {
        return -ENODEV;
    }
        count = n / sizeof(int);
        if (n % sizeof(int) || count % 2 == 0)
                return -EINVAL;
        wbuf = memdup_user(buf, n);
        if (IS_ERR(wbuf))
                return PTR_ERR(wbuf);
        spin_lock_irqsave(&lock, flags);
    dprintk("lirc_write called, offset %d",TX_OFFSET_GPIOCHIP);
        for (i = 0; i < count; i++) {
                if (i%2)
                        send_space(wbuf[i] - delta);
                else
                        delta = send_pulse(wbuf[i]);
        }
        gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, invert);
        spin_unlock_irqrestore(&lock, flags);
    if (count>11) {
        dprintk("lirc_write sent %d pulses: no10: %d, no11: %d\n",count,wbuf[10],wbuf[11]);
    }
        kfree(wbuf);
        return n;
}

/* interpret lirc commands */
static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
        int result;
        __u32 value;

        switch (cmd) {
        case LIRC_GET_SEND_MODE:
            return -ENOIOCTLCMD;
            break;

        case LIRC_SET_SEND_MODE:
            result = get_user(value, (__u32 *) arg);
            if (result)
                return result;
            /* only LIRC_MODE_PULSE supported */
            if (value != LIRC_MODE_PULSE)
                return -ENOSYS;
            dprintk("Sending stuff on lirc");
            break;

        case LIRC_GET_LENGTH:
            return -ENOSYS;
            break;

        case LIRC_SET_SEND_DUTY_CYCLE:
            result = get_user(value, (__u32 *) arg);
            if (result)
                return result;
            if (value <= 0 || value > 100)
                return -EINVAL;
            dprintk("SET_SEND_DUTY_CYCLE to %d \n", value);
            return init_timing_params(value, freq);
            break;

        case LIRC_SET_SEND_CARRIER:
            result = get_user(value, (__u32 *) arg);
            if (result)
                return result;
            if (value > 500000 || value < 20000)
                return -EINVAL;
            dprintk("SET_SEND_CARRIER to %d \n",value);
            return init_timing_params(duty_cycle, value);
            break;

        default:
            return lirc_dev_fop_ioctl(filep, cmd, arg);
        }
        return 0;
}

static const struct file_operations lirc_fops = {
        .owner                = THIS_MODULE,
        .write                = lirc_write,
        .unlocked_ioctl        = lirc_ioctl,
        .read                = lirc_dev_fop_read, // this and the rest is default
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
static struct platform_driver lirc_gpio_driver = {
        .driver = {
                .name   = LIRC_DRIVER_NAME,
                .owner  = THIS_MODULE,
        },
};



/* stuff for sysfs*/

static DEFINE_MUTEX(sysfs_lock);



static ssize_t lirc_txpin_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",gpio_out_pin);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_txpin_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int new_pin;
    ssize_t status;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&new_pin);
    status = setup_tx(new_pin) ? : size;
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_rxpin_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",gpio_in_pin);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_rxpin_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int new_pin;
    ssize_t status;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&new_pin);
    status = setup_rx(new_pin) ? : size;
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_softcarrier_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",softcarrier);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_softcarrier_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int try_value;
    ssize_t status=size;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&try_value);
    if ((try_value==0) || (try_value==1)) {
        softcarrier=try_value;
    }
    else
        status = -EINVAL;
    mutex_unlock(&sysfs_lock);
    return status;
}
static ssize_t lirc_invert_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",invert);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_invert_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int try_value;
    ssize_t status=size;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&try_value);
    if ((try_value==0) || (try_value==1)) {
        invert=try_value;
    }
    else
        status = -EINVAL;
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_sense_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",sense);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_sense_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int try_value;
    ssize_t status=size;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&try_value);
    mutex_unlock(&sysfs_lock);
    if ((try_value<2) && (try_value>-2)) {
        sense=try_value;
    }
    else
        return -EINVAL;

    if (gpio_in_pin>0) {
        set_sense();
    }
    return status;
}

/* I don't think we need another device, so just put it in the class directory
 * All we need is a way to access some global parameters of this module */

static struct class_attribute lirc_gpio_attrs[] = {
    __ATTR(tx_gpio_pin, 0644, lirc_txpin_show, lirc_txpin_store),
    __ATTR(rx_gpio_pin, 0644, lirc_rxpin_show, lirc_rxpin_store),
    __ATTR(lirc_softcarrier, 0644, lirc_softcarrier_show, lirc_softcarrier_store),
    __ATTR(lirc_invert, 0644, lirc_invert_show, lirc_invert_store),
    __ATTR(lirc_sense, 0644, lirc_sense_show, lirc_sense_store),
    __ATTR_NULL,
};
static struct class lirc_gpio_class = {
    .name = "lirc_gpio",
    .owner = THIS_MODULE,
    .class_attrs = lirc_gpio_attrs,
};

/* end of sysfs stuff */

/* initialize / free THIS driver and device and a lirc buffer*/

static int __init lirc_gpio_init(void)
{
        int result;

        /* Init read buffer. */
        result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
        if (result < 0)
                return -ENOMEM;

        result = platform_driver_register(&lirc_gpio_driver);
        if (result) {
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": lirc register returned %d\n", result);
                goto exit_buffer_free;
        }

        lirc_gpio_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
        if (!lirc_gpio_dev) {
                result = -ENOMEM;
                goto exit_driver_unregister;
        }

        result = platform_device_add(lirc_gpio_dev);
        if (result)
                goto exit_device_put;

        return 0;

exit_device_put:
        platform_device_put(lirc_gpio_dev);

exit_driver_unregister:
        platform_driver_unregister(&lirc_gpio_driver);

exit_buffer_free:
        lirc_buffer_free(&rbuf);

        return result;
}

static void lirc_gpio_exit(void)
{
    setup_tx(0); // frees gpio_out_pin if set
    setup_rx(0); //  dito
        platform_device_unregister(lirc_gpio_dev);
        platform_driver_unregister(&lirc_gpio_driver);
        lirc_buffer_free(&rbuf);
}

/* end of stuff for THIS driver/device registration */
/* ignorance of unset pins in setup routines tolerate call if nothing is set up */

/* master init */

static int __init lirc_gpio_init_module(void)
{
    int result,temp_in_pin,temp_out_pin;

        result = lirc_gpio_init();
        if (result)
                return result;
    // 'driver' is the lirc driver
        driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
    LIRC_CAN_SET_SEND_CARRIER |
    LIRC_CAN_SEND_PULSE |
    LIRC_CAN_REC_MODE2;

        driver.dev = &lirc_gpio_dev->dev;  // link THIS platform device to lirc driver
        driver.minor = lirc_register_driver(&driver);

        if (driver.minor < 0) {
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": device registration failed with %d\n", result);
                result = -EIO;
                goto exit_lirc;
        }

        printk(KERN_INFO LIRC_DRIVER_NAME ": driver registered!\n");

        result = set_gpiochip();
        if (result < 0)
                goto exit_lirc;
    /* some hacking to get pins initialized on first used */
    /* setup_tx/rx will not do anything if pins would not change */
    temp_out_pin = gpio_out_pin; gpio_out_pin = 0;
    result = setup_tx(temp_out_pin);
    if (result < 0)
        goto exit_lirc;
    /* dito for rx */
    temp_in_pin = gpio_in_pin; gpio_in_pin = 0;
    result = setup_rx(temp_in_pin);
    if (result < 0)
        goto exit_lirc;
    if (device_open) {  // this is unlikely, but well...
        result = set_use_inc((void*) 0);
        if (result<0) {
            goto exit_lirc;
        }
    }

    result=class_register(&lirc_gpio_class);
    if (result) {
        goto exit_lirc;
    }


        return 0;

exit_lirc:
    /* failed attempt to setup_tx/rx sets pin to 0. */
    /* next call with arg 0 will then not do anything -> only one exit routine */
        lirc_gpio_exit();

        return result;
}

static void __exit lirc_gpio_exit_module(void)
{

        lirc_gpio_exit();
    class_unregister(&lirc_gpio_class);

        lirc_unregister_driver(driver.minor);
        printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");
}

module_init(lirc_gpio_init_module);
module_exit(lirc_gpio_exit_module);

MODULE_DESCRIPTION("Infra-red receiver and blaster driver for GPIO-Lib.");
MODULE_DESCRIPTION("Parameters can be set/changed in /sys/class/lirc_gpio");
MODULE_DESCRIPTION("If RX Pin is changed, previous value of sense is taken");
MODULE_DESCRIPTION("You may want to write -1 to lirc_sense to force new auto-detection");
MODULE_AUTHOR("Matthias Hoelling <mhoel....@gmail.nospam.com");
MODULE_LICENSE("GPL");

module_param(gpio_out_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_out_pin, "GPIO output/transmitter pin array");

module_param(gpio_in_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_in_pin, "GPIO input/receiver pin array");

module_param(sense, int, S_IRUGO);
MODULE_PARM_DESC(sense, "Override autodetection of IR receiver circuit");

module_param(softcarrier, int, S_IRUGO);
MODULE_PARM_DESC(softcarrier, "Software carrier (0 = off, 1 = on, default on)");

module_param(invert, int, S_IRUGO);
MODULE_PARM_DESC(invert, "Invert output (0 = off, 1 = on, default off");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
