#!/bin/bash

set -e

BPIBOARD=bpi-m1
MODE=$1
TYPE=$2

generate_board_config() {
        cat <<-EOT
	CONFIG_ARMV7_BOOT_SEC_DEFAULT=y
	CONFIG_OLD_SUNXI_KERNEL_COMPAT=y
	CONFIG_BPI_CHIP="a20"
	CONFIG_BPI_BOARD="${BPIBOARD}"
	CONFIG_BPI_SERVICE="linux"
	CONFIG_BPI_PATH="bananapi/${BPIBOARD}/linux/"
	CONFIG_BPI_UENVFILE="bananapi/${BPIBOARD}/linux/uEnv.txt"
	EOT
}

generate_lcd_config() {
        cat <<-EOT
	CONFIG_VIDEO_LCD_MODE="x:800,y:480,depth:24,pclk_khz:30000,le:40,ri:40,up:29,lo:13,hs:48,vs:3,sync:3,vmode:0"
	CONFIG_VIDEO_LCD_POWER="PH12"
	CONFIG_VIDEO_LCD_BL_EN="PH9"
	CONFIG_VIDEO_LCD_BL_PWM="PB2"
	CONFIG_VIDEO_LCD_BL_PWM_ACTIVE_LOW=y
	EOT
}

export BOARD=${BPIBOARD}
export ARCH=arm
BPICONF=Bananapi_defconfig
NEWBPICONF=$BPICONF

if [ -z "$MODE" ]; then
	MODE=mainline
else
	MODE=legacy
	export BOARD=${BPIBOARD}-legacy
	NEWBPICONF=BPI-$BPICONF
	cat configs/$BPICONF >configs/$NEWBPICONF
	generate_board_config "$1" >> configs/$NEWBPICONF
fi

if [ "$2" = "lcd" ]; then
	TYPE=lcd
	generate_lcd_config "$1" >> configs/$NEWBPICONF
fi

echo MODE=$MODE
echo TYPE=$TYPE

KBUILD_OUTPUT=out/$BOARD
export KBUILD_OUTPUT
mkdir -p $KBUILD_OUTPUT
export CROSS_COMPILE=arm-linux-gnueabihf-
make $NEWBPICONF
make -j8
./bpi-uimgz.sh $BOARD

