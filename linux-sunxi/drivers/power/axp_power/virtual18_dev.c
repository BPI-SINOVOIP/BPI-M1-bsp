/*
 * drivers/power/axp_power/virtual18_dev.c
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <mach/irqs.h>
#include <linux/power_supply.h>
#include <linux/mfd/axp-mfd.h>


#include "axp-cfg.h"


static struct platform_device virt[]={
	{
			.name = "reg-18-cs-ldo2",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_analog/fm",
			}
 	},{
			.name = "reg-18-cs-ldo3",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_flash",
			}
 	},{
			.name = "reg-18-cs-ldo4",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_spdif",
			}
 	},{
			.name = "reg-18-cs-ldo5",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_others",
			}
 	},{
			.name = "reg-18-cs-buck1",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_io",
			}
 	},{
			.name = "reg-18-cs-buck2",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_core",
			}
 	},{
			.name = "reg-18-cs-buck3",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_memory",
			}
	},{
			.name = "reg-18-cs-sw1",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_sdram",
			}
	},{
			.name = "reg-18-cs-sw2",
			.id = -1,
			.dev		= {
				.platform_data = "axp18_sdcard",
			}
	},
};



 static int __init virtual_init(void)
{
	int j,ret;
	for (j = 0; j < ARRAY_SIZE(virt); j++){
 		ret =  platform_device_register(&virt[j]);
  		if (ret)
				goto creat_devices_failed;
	}

	return ret;

creat_devices_failed:
	while (j--)
		platform_device_register(&virt[j]);
	return ret;

}

module_init(virtual_init);

static void __exit virtual_exit(void)
{
	int j;
	for (j = ARRAY_SIZE(virt) - 1; j >= 0; j--){
		platform_device_unregister(&virt[j]);
	}
}
module_exit(virtual_exit);

MODULE_DESCRIPTION("Krosspower axp regulator test");
MODULE_AUTHOR("Donglu Zhang Krosspower");
MODULE_LICENSE("GPL");