/*
*********************************************************************************************************
*                                                    LINUX-KERNEL
*                                        newbie Linux Platform Develop Kits
*                                                   Kernel Module
*
*                                    (c) Copyright 2006-2011, kevin.z China
*                                             All Rights Reserved
*
* File    : standby_power.c
* By      : kevin.z
* Version : v1.0
* Date    : 2011-5-31 14:34
* Descript:
* Update  : date                auther      ver     notes
*********************************************************************************************************
*/
#include "standby_i.h"

#ifdef  CONFIG_ARCH_SUN7I
#define NMI_CTL_REG            (0xf1c00030)
#define NMI_IRG_PENDING_REG    (0xf1c00034)
#define NMI_INT_ENABLE_REG     (0xf1c00038)
#endif
#define writel(v, addr)	(*((volatile unsigned long  *)(addr)) = (unsigned long)(v))


//==============================================================================
// POWER CHECK FOR SYSTEM STANDBY
//==============================================================================


/*
*********************************************************************************************************
*                           standby_power_init
*
* Description: init power for standby.
*
* Arguments  : none;
*
* Returns    : result;
*********************************************************************************************************
*/
__s32 standby_power_init(__u32 wakeup_src)
{
	__u8	reg_val;

	standby_twi_init(AXP_IICBUS);

	if(wakeup_src & AXP_WAKEUP_KEY){
		/* enable pek long/short */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN3, &reg_val);
		reg_val |= 0x03;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN3, &reg_val);
	}

	if(wakeup_src & AXP_WAKEUP_LONG_KEY){
		/* enable pek long */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN3, &reg_val);
		reg_val |= 0x01;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN3, &reg_val);
		/*pek long period setting: 1s*/
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_PEK, &reg_val);
		reg_val &= 0xcf;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_PEK, &reg_val);
	}
	
	if(wakeup_src & AXP_WAKEUP_SHORT_KEY){
		/* enable pek short */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN3, &reg_val);
		reg_val |= 0x02;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN3, &reg_val);
	}

	if(wakeup_src & AXP_WAKEUP_DESCEND){
		/* enable pek desend trigger */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN5, &reg_val);
		reg_val |= 0x20;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN5, &reg_val);
	}

	if(wakeup_src & AXP_WAKEUP_ASCEND){
		/* enable pek ascend trigger */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN5, &reg_val);
		reg_val |= 0x40;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN5, &reg_val);
	}
	
	if(wakeup_src & AXP_WAKEUP_LOWBATT){
		/* enable low voltage warning */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN4, &reg_val);
		reg_val |= 0x03;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN4, &reg_val);
		/* clear pending */
		reg_val |= 0x03;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQ4, &reg_val);
	}
	
	if(wakeup_src & AXP_WAKEUP_USB){
		/* enable usb plug-in / plug-out */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN1, &reg_val);
		reg_val |= 0x03<<2;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN1, &reg_val);
	}

	if(wakeup_src & AXP_WAKEUP_AC){
		/* enable ac plug-in / plug-out */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN1, &reg_val);
		reg_val |= 0x03<<5;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN1, &reg_val);
	}

#ifdef CONFIG_ARCH_SUN7I
    writel(0x1,NMI_INT_ENABLE_REG);
#endif
    
    
    return 0;
}


/*
*********************************************************************************************************
*                           standby_power_exit
*
* Description: exit power for standby.
*
* Arguments  : none;
*
* Returns    : result;
*********************************************************************************************************
*/
__s32 standby_power_exit(__u32 wakeup_src)
{
	__u8    reg_val;

	twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQ4, &reg_val);
	twi_byte_rw(TWI_OP_WR, AXP_ADDR,0x0E, &reg_val);

	if(wakeup_src & AXP_WAKEUP_KEY){
		/* disable pek long/short */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN3, &reg_val);
		reg_val &= ~0x03;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN3, &reg_val);
	}
	
	if(wakeup_src & AXP_WAKEUP_LONG_KEY){
		/* enable pek long/short */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN3, &reg_val);
		reg_val &= ~0x01;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN3, &reg_val);
	}
	
	if(wakeup_src & AXP_WAKEUP_SHORT_KEY){
		/* enable pek long/short */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN3, &reg_val);
		reg_val &= ~0x02;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN3, &reg_val);
	}

	if(wakeup_src & AXP_WAKEUP_DESCEND){
		/* disable pek desend trigger */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN5, &reg_val);
		reg_val &= ~0x20;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN5, &reg_val);
	}

	if(wakeup_src & AXP_WAKEUP_ASCEND){
		/* disable pek desend trigger */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN5, &reg_val);
		reg_val &= ~0x40;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN5, &reg_val);
	}
	
	if(wakeup_src & AXP_WAKEUP_LOWBATT){
		/* disable low voltage warning */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN4, &reg_val);
		reg_val &= ~0x03;
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN4, &reg_val);
	}

	if(wakeup_src & AXP_WAKEUP_USB){
		/* disable usb plug-in / plug-out */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN1, &reg_val);
		reg_val &= ~(0x03<<2);
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN1, &reg_val);
	}

	if(wakeup_src & AXP_WAKEUP_AC){
		/* disable ac plug-in / plug-out */
		twi_byte_rw(TWI_OP_RD, AXP_ADDR,AXP20_IRQEN1, &reg_val);
		reg_val &= ~(0x03<<5);
		twi_byte_rw(TWI_OP_WR, AXP_ADDR,AXP20_IRQEN1, &reg_val);
	}

#ifdef CONFIG_ARCH_SUN7I
    writel(0x0,NMI_INT_ENABLE_REG);
#endif
    standby_twi_exit();
    return 0;
}


static inline int check_range(struct axp_info *info,__s32 voltage)
{
	if (voltage < info->min_uV || voltage > info->max_uV)
		return -1;

	return 0;
}

static int axp20_ldo4_data[] = {
    1250, 1300, 1400, 1500, 1600, 1700,
    1800, 1900, 2000, 2500, 2700, 2800,
    3000, 3100, 3200, 3300
};

static struct axp_info axp20_info[] = {
	AXP(POWER_VOL_LDO1,	 AXP20LDO1,	AXP20LDO1,	  0, AXP20_LDO1,  0, 0),//ldo1 for rtc
	AXP(POWER_VOL_LDO2,	      1800,      3300,  100, AXP20_LDO2,  4, 4),//ldo2 for analog1
	AXP(POWER_VOL_LDO3,	       700,      3500,   25, AXP20_LDO3,  0, 7),//ldo3 for digital
	AXP(POWER_VOL_LDO4,	      1250,      3300,  100, AXP20_LDO4,  0, 4),//ldo4 for analog2
	AXP(POWER_VOL_DCDC2,       700,      2275,   25, AXP20_BUCK2, 0, 6),//buck2 for core
	AXP(POWER_VOL_DCDC3,       700,      3500,   25, AXP20_BUCK3, 0, 7),//buck3 for memery
};

static inline struct axp_info *find_info(int id)
{
	struct axp_info *ri;
	int i;

	for (i = 0; i < sizeof(axp20_info)/sizeof(struct axp_info); i++) {
		ri = &axp20_info[i];
		if (ri->id == id)
			return ri;
	}
	return 0;
}

/*
*********************************************************************************************************
*                           standby_set_voltage
*
*Description: set voltage for standby;
*
*Arguments  : type      voltage type, defined as "enum power_vol_type_e";
*             voltage   voltage value, based on "mv";
*
*Return     : none;
*
*Notes      :
*
*********************************************************************************************************
*/
void  standby_set_voltage(enum power_vol_type_e type, __s32 voltage)
{
	struct axp_info *info = 0;
	__u8 val, mask, reg_val;

	info = find_info(type);
	if (info == 0) {
		return;
	}

	if (check_range(info, voltage)) {
		return;
	}

	if (type != POWER_VOL_LDO4)
		val = raw_lib_udiv((voltage-info->min_uV+info->step_uV-1), info->step_uV);
	else{
		if(voltage == 1250000 ){
			val = 0;
		}
		else{
			val = raw_lib_udiv((voltage-1200000+info->step_uV-1), info->step_uV);
			if(val > 16){
				val = val - 6;
			}
			else if(val > 13){
				val = val - 5;
			}
			else if(val > 12){
				val = val - 4;
			}
			else if(val > 8)
				val = 8;
		}
	}


	val <<= info->vol_shift;
	mask = ((1 << info->vol_nbits) - 1)  << info->vol_shift;

	twi_byte_rw(TWI_OP_RD,AXP_ADDR,info->vol_reg, &reg_val);

	if ((reg_val & mask) != val) {
		reg_val = (reg_val & ~mask) | val;
		twi_byte_rw(TWI_OP_WR,AXP_ADDR,info->vol_reg, &reg_val);
	}

	return;
}


/*
*********************************************************************************************************
*                           standby_get_voltage
*
*Description: get voltage for standby;
*
*Arguments  : type  voltage type, defined as "enum power_vol_type_e";
*
*Return     : voltage value, based on "mv";
*
*Notes      :
*
*********************************************************************************************************
*/
__u32 standby_get_voltage(enum power_vol_type_e type)
{
	struct axp_info *info = 0;
	__u8 val, mask;

	info = find_info(type);
	if (info == 0) {
		return -1;
	}

	twi_byte_rw(TWI_OP_RD,AXP_ADDR,info->vol_reg, &val);

	mask = ((1 << info->vol_nbits) - 1)  << info->vol_shift;
	val = (val & mask) >> info->vol_shift;
	if (type != POWER_VOL_LDO4)
		return info->min_uV + info->step_uV * val;
	else
		return axp20_ldo4_data[val]*1000;
}


