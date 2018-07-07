/*
 * LED Disk Activity Trigger
 *
 * Copyright 2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/leds.h>

static void ledtrig_disk_timerfunc(unsigned long data);

DEFINE_LED_TRIGGER(ledtrig_disk);
static DEFINE_TIMER(ledtrig_disk_timer, ledtrig_disk_timerfunc, 0, 0);
static int disk_activity;
static int disk_lastactivity;

void ledtrig_disk_activity(void)
{
	disk_activity++;
	if (!timer_pending(&ledtrig_disk_timer))
		mod_timer(&ledtrig_disk_timer, jiffies + msecs_to_jiffies(10));
}
EXPORT_SYMBOL(ledtrig_disk_activity);

static void ledtrig_disk_timerfunc(unsigned long data)
{
	if (disk_lastactivity != disk_activity) {
		disk_lastactivity = disk_activity;
		/* INT_MAX will set each LED to its maximum brightness */
		led_trigger_event(ledtrig_disk, INT_MAX);
		mod_timer(&ledtrig_disk_timer, jiffies + msecs_to_jiffies(10));
	} else {
		led_trigger_event(ledtrig_disk, LED_OFF);
	}
}

static int __init ledtrig_disk_init(void)
{
	led_trigger_register_simple("disk-activity", &ledtrig_disk);
	return 0;
}

static void __exit ledtrig_disk_exit(void)
{
	led_trigger_unregister_simple(ledtrig_disk);
}

module_init(ledtrig_disk_init);
module_exit(ledtrig_disk_exit);

MODULE_AUTHOR("Richard Purdie <rpurdie@openedhand.com>");
MODULE_DESCRIPTION("LED IDE Disk Activity Trigger");
MODULE_LICENSE("GPL");
