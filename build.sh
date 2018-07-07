#!/bin/bash
# (c) 2015, Leo Xu <otakunekop@banana-pi.org.cn>
# Build script for BPI-M2P-BSP 2016.03.02

T=
MACH="sun7i"
BOARD=$1
board=
kernel=
BOOT_PACK_P=
MODE=$2

echo "top dir $T"

cp_download_files()
{
SD="$T/SD/${board}"
U="${SD}/100MB"
B="${SD}/BPI-BOOT"
R="${SD}/BPI-ROOT"
	#
	## clean SD dir.
	#
	rm -rf $SD
	#
	## create SD dirs (100MB, BPI-BOOT, BPI-ROOT) 
	#
	mkdir -p $SD
	mkdir -p $U
	mkdir -p $B
	mkdir -p $R
	#
	## copy files to 100MB
	#
	#cp -a $T/output/100MB/* $U
	cp -a $T/u-boot-sunxi/out/*.img.gz $U
	#
	## copy files to BPI-BOOT
	#
	mkdir -p $B/bananapi
	cp -a ${BOOT_PACK_P}/* $B/bananapi

	#
	## copy files to BPI-ROOT
	#
        case $BOARD in
        BPI-M1-M1P-R1*)
	  mkdir -p $R/usr/lib/u-boot/bananapi/bpi-m1
	  mkdir -p $R/usr/lib/u-boot/bananapi/bpi-m1p
	  mkdir -p $R/usr/lib/u-boot/bananapi/bpi-r1
	  cp -a $U/*bpi-m1-*.gz $R/usr/lib/u-boot/bananapi/bpi-m1/
	  cp -a $U/*bpi-m1p-*.gz $R/usr/lib/u-boot/bananapi/bpi-m1p/
	  cp -a $U/*bpi-r1-*.gz $R/usr/lib/u-boot/bananapi/bpi-r1/
	  cp -a $T/linux-sunxi/arch/arm/boot/uImage $B/bananapi/chip/a20/uImage.sun7i
          ;;
        *)
	  mkdir -p $R/usr/lib/u-boot/bananapi/${board}
	  cp -a $U/*${board}-*.gz $R/usr/lib/u-boot/bananapi/${board}
	  cp -a $T/linux-sunxi/arch/arm/boot/uImage $B/bananapi/${board}/linux/uImage
          ;;
        esac
	rm -rf $R/lib/modules
	mkdir -p $R/lib/modules
	cp -a $T/linux-sunxi/output/lib/modules/${kernel} $R/lib/modules
	#
	## create files for bpi-tools & bpi-migrate
	#
	(cd $B ; tar czvf $SD/BPI-BOOT-${board}.tgz .)
	(cd $R ; tar czvf $SD/${kernel}.tgz lib/modules)
	(cd $R ; tar czvf $SD/BOOTLOADER-${board}.tgz usr/lib/u-boot/bananapi)

	return #SKIP
}

list_boards() {
	cat <<-EOT
	NOTICE:
	new build.sh default select $BOARD and pack all boards
	supported boards:
	EOT
	(cd sunxi-pack/chips/$MACH/configs ; ls -1d BPI* )
	echo
}

list_boards

if [ -z "$BOARD" ]; then
        echo -e "\033[31mNo BOARD select\033[0m"
	echo
	echo -e "\033[31m# ./build.sh <board> <mode> \033[0m"
	exit 1
fi

./configure $BOARD

if [ -f env.sh ] ; then
	. env.sh
fi

T="$TOPDIR"
case $BOARD in
  BPI-M1-M1P-R1*)
    board="bpi-m1-m1p-r1"
    kernel="3.4.112-BPI-M1-Kernel"
    BOOT_PACK_P=$T/sunxi-pack/chips/${MACH}/configs/${BOARD}/bananapi
    ;;
  *)
    board=$(echo $BOARD | tr '[A-Z]' '[a-z]')
    kernel="3.4.112-${BOARD}-Kernel"
    BOOT_PACK_P=$T/sunxi-pack/chips/${MACH}/configs/${BOARD}/linux
    ;;
esac

echo "This tool support following building mode(s):"
echo "--------------------------------------------------------------------------------"
echo "	1. Build all, uboot and kernel and pack to download images."
echo "	2. Build uboot only."
echo "	3. Build kernel only."
echo "	4. kernel configure."
echo "	5. Pack the builds to target download image, this step must execute after u-boot,"
echo "	   kernel and rootfs build out"
echo "	6. update files for SD"
echo "	7. Clean all build."
echo "--------------------------------------------------------------------------------"

if [ -z "$MODE" ]; then
	read -p "Please choose a mode(1-7): " mode
	echo
else
	mode=1
fi

if [ -z "$mode" ]; then
        echo -e "\033[31m No build mode choose, using Build all default   \033[0m"
        mode=1
fi

echo -e "\033[31m Now building...\033[0m"
echo
case $mode in
	1) make && 
	   make pack && 
	   cp_download_files;;
	2) make u-boot;;
	3) make kernel;;
	4) make kernel-config;;
	5) make pack;;
	6) cp_download_files;;
	7) make clean;;
esac
echo

echo -e "\033[31m Build success!\033[0m"
echo
