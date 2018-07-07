/*
 * a20-tp.c - A20 SoC Resistive Touch Panel (RTP) embedded sensor driver only
 *
 * Copyright (C) 2013 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * Datasheet: http://dl.linux-sunxi.org/A20/A20%20User%20Manual%202013-03-22.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>

#define IRQ_TP                 29
#define TP_BASSADDRESS         0xf1c25000
#define TP_CTRL0               0x00
#define TP_CTRL1               0x04
#define TP_CTRL2               0x08
#define TP_CTRL3               0x0c
#define TP_INT_FIFOC           0x10
#define TP_INT_FIFOS           0x14
#define TP_TPR                 0x18
#define TP_CDAT                0x1c
#define TEMP_DATA              0x20
#define TP_DATA                0x24

#define ADC_CLK_DIVIDER        (0x2<<20)
#define CLK                    7
#define FS_DIV                 (CLK<<16)
#define ACQ                    0x3f
#define T_ACQ                  ACQ

#define TP_CTRL_MODE_EN             (1<<4)

#define TP_DATA_IRQ_EN         (1<<16)
#define TP_FIFO_FLUSH          (1<<4)
#define TP_TEMP_IRQ_EN		(1<<18)
#define TP_TEMP_IRQ_PENDING	(1<<18)

#define FIFO_DATA_PENDING      (1<<16)
#define TP_UP_PENDING          (1<<1)
#define TP_DOWN_PENDING        (1<<0)

#define TP_TPR_TEMP_ENABLE	(1<<16)
#define TP_TPR_TEMP_PERIOD	0x0fff

#define TP_TEMP_NODATA		-666

struct tp_data {
	struct resource *res;
	void __iomem *base_addr;
	int irq;
	s16 temperature;
	struct device *hwmon_dev;
};

/* Since we need to wait for an irq to have the temperature, we cannot give
 * temperature for a short period of time.
 * So until we have a temperature we return -EAGAIN */
static ssize_t
show_temp(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct tp_data *data = dev_get_drvdata(dev);
	if (attr->index == 1)
		return sprintf(buf, "264800\n");
	if (attr->index == 2)
		return sprintf(buf, "-144700\n");
	if (attr->index == 3)
		return sprintf(buf, "a20_tp\n");
	if (data->temperature == TP_TEMP_NODATA)
		return -EAGAIN;
	return sprintf(buf, "%d\n", data->temperature * 100);
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_max, S_IRUGO, show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_min, S_IRUGO, show_temp, NULL, 2);
static SENSOR_DEVICE_ATTR(name, S_IRUGO, show_temp, NULL, 3);

static struct attribute *tp_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_name.dev_attr.attr,
	NULL
};

static const struct attribute_group tp_group = {
	.attrs = tp_attributes,
};

static irqreturn_t tp_handle_irq(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct tp_data *ts_data = (struct tp_data *)platform_get_drvdata(pdev);
	unsigned int reg_val;

	reg_val = readl(TP_BASSADDRESS + TP_INT_FIFOS);
	if (!(reg_val & (TP_DOWN_PENDING | FIFO_DATA_PENDING | TP_UP_PENDING))) {
		if (reg_val & TP_TEMP_IRQ_PENDING) {
			writel(reg_val & TP_TEMP_IRQ_PENDING, TP_BASSADDRESS + TP_INT_FIFOS);

			reg_val = readl(TP_BASSADDRESS + TEMP_DATA);
			reg_val &= 0x00000FFF;/* 12 bit wide */
			ts_data->temperature = reg_val - 1447;
			return IRQ_HANDLED;
		}
		return IRQ_NONE;
	}
	writel(reg_val, TP_BASSADDRESS + TP_INT_FIFOS);
	return IRQ_HANDLED;
}

static int __devinit a20_tp_hwmon_probe(struct platform_device *pdev)
{
	int err = 0;
	int irq = platform_get_irq(pdev, 0);
	struct tp_data *ts_data;

	ts_data = kzalloc(sizeof(struct tp_data), GFP_KERNEL);
	if (!ts_data) {
		dev_err(&pdev->dev, "Cannot allocate driver structures\n");
		err = -ENOMEM;
		return err;
	}

	ts_data->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ts_data->res) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "Cannot get the MEMORY\n");
		goto label_err1;
	}

	ts_data->base_addr = (void __iomem *)TP_BASSADDRESS;

	ts_data->irq = irq;

	err = request_irq(irq, tp_handle_irq,
			IRQF_DISABLED, pdev->name, pdev);
	if (err) {
		dev_err(&pdev->dev, "Cannot request keypad IRQ\n");
		goto label_err1;
	}

	platform_set_drvdata(pdev, ts_data);

	ts_data->temperature = TP_TEMP_NODATA;

	writel(ADC_CLK_DIVIDER | FS_DIV | T_ACQ, TP_BASSADDRESS + TP_CTRL0);

	/* Enable the temperature IRQ */
	writel(TP_TEMP_IRQ_EN, TP_BASSADDRESS + TP_INT_FIFOC);

	/* Enable the temperature */
	writel(TP_TPR_TEMP_ENABLE | TP_TPR_TEMP_PERIOD, TP_BASSADDRESS + TP_TPR);

	/* Enable TP */
	writel(TP_CTRL_MODE_EN, TP_BASSADDRESS + TP_CTRL1);

	err = sysfs_create_group(&pdev->dev.kobj, &tp_group);
	if (err) {
		dev_err(&pdev->dev, "Cannot create sysfs group\n");
		goto label_err2;
	}

	ts_data->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(ts_data->hwmon_dev)) {
		err = PTR_ERR(ts_data->hwmon_dev);
		goto label_err3;
	}
	dev_info(&pdev->dev, "TP initialization success\n");
	return 0;
label_err3:
	sysfs_remove_group(&pdev->dev.kobj, &tp_group);
label_err2:
	free_irq(ts_data->irq, pdev);
label_err1:
	kfree(ts_data);
	return err;
}

static int __devexit a20_tp_hwmon_remove(struct platform_device *pdev)
{
	struct tp_data *ts_data = platform_get_drvdata(pdev);

	/* desactivate all IRQ */
	writel(0, ts_data->base_addr + TP_INT_FIFOC);

	hwmon_device_unregister(ts_data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &tp_group);

	free_irq(ts_data->irq, pdev);
	kfree(ts_data);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver a20_tp_hwmon_driver = {
	.probe		= a20_tp_hwmon_probe,
	.remove		= __devexit_p(a20_tp_hwmon_remove),
	.driver		= {
		.name	= "a20-tp-hwmon",
	},
};

static struct resource a20_tp_hwmon_resource[] = {
	{
		.flags  = IORESOURCE_IRQ,
		.start  = SW_INT_IRQNO_TOUCH_PANEL ,
		.end    = SW_INT_IRQNO_TOUCH_PANEL ,
	},
	{
		.flags	= IORESOURCE_MEM,
		.start	= TP_BASSADDRESS,
		.end	= TP_BASSADDRESS + 0x100-1,
	},
};

struct platform_device a20_tp_hwmon_device = {
	.name		= "a20-tp-hwmon",
	.id		    = -1,
	.resource	= a20_tp_hwmon_resource,
	.num_resources	= ARRAY_SIZE(a20_tp_hwmon_resource),
};

static int __init a20_tp_hwmon_init(void)
{
	platform_device_register(&a20_tp_hwmon_device);
	return platform_driver_register(&a20_tp_hwmon_driver);
}

static void __exit a20_tp_hwmon_exit(void)
{
	platform_driver_unregister(&a20_tp_hwmon_driver);
	platform_device_unregister(&a20_tp_hwmon_device);
}

module_init(a20_tp_hwmon_init);
module_exit(a20_tp_hwmon_exit);

MODULE_AUTHOR("Corentin LABBE");
MODULE_DESCRIPTION("A20 SoC embedded touchscreen hwmon driver");
MODULE_LICENSE("GPL");

