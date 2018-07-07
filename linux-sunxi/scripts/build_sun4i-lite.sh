#!/bin/bash
set -e
#########################################################################
#
#          Simple build scripts to build krenel(with rootfs)  -- by Benn
#
#########################################################################


#Setup common variables
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabi-
export AS=${CROSS_COMPILE}as
export LD=${CROSS_COMPILE}ld
export CC=${CROSS_COMPILE}gcc
export AR=${CROSS_COMPILE}ar
export NM=${CROSS_COMPILE}nm
export STRIP=${CROSS_COMPILE}strip
export OBJCOPY=${CROSS_COMPILE}objcopy
export OBJDUMP=${CROSS_COMPILE}objdump

KERNEL_VERSION="3.0"
LICHEE_KDIR=`pwd`
LICHEE_MOD_DIR=${LICHEE_KDIR}/output/lib/modules/${KERNEL_VERSION}

update_kern_ver()
{
	if [ -r include/generated/utsrelease.h ]; then
		KERNEL_VERSION=`cat include/generated/utsrelease.h |awk -F\" '{print $2}'`
	fi
	LICHEE_MOD_DIR=${LICHEE_KDIR}/output/lib/modules/${KERNEL_VERSION}
}

show_help()
{
	printf "
Build script for Lichee platform

Invalid Options:

	help         - show this help
	kernel       - build kernel
	modules      - build kernel module in modules dir
	clean        - clean kernel and modules

"
}

build_kernel()
{
	if [ ! -e .config ]; then
		echo -e "\n\t\tUsing default config... ...!\n"
		cp arch/arm/configs/sun4i_defconfig .config
	fi

	make ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -j8 uImage modules

	update_kern_ver

	if [ -d output ]; then
		rm -rf output
	fi
	mkdir -p $LICHEE_MOD_DIR

	${OBJCOPY} -R .note.gnu.build-id -S -O binary vmlinux output/bImage
	cp -vf arch/arm/boot/[zu]Image output/
	cp .config output/


	for file in $(find drivers sound crypto block fs security net -name "*.ko"); do
		cp $file ${LICHEE_MOD_DIR}
	done
	cp -f Module.symvers modules.* ${LICHEE_MOD_DIR}

	#copy bcm4330 firmware and nvram.txt
	cp drivers/net/wireless/bcm4330/firmware/bcm4330.bin ${LICHEE_MOD_DIR}
	cp drivers/net/wireless/bcm4330/firmware/bcm4330.hcd ${LICHEE_MOD_DIR}
	cp drivers/net/wireless/bcm4330/firmware/nvram.txt ${LICHEE_MOD_DIR}/bcm4330_nvram.txt
}

build_modules()
{
	echo "Building modules"

	if [ ! -f include/generated/utsrelease.h ]; then
		printf "Please build kernel first!\n"
		exit 1
	fi

	update_kern_ver

	make -C modules/example LICHEE_MOD_DIR=${LICHEE_MOD_DIR} LICHEE_KDIR=${LICHEE_KDIR} \
		install

	(
	export LANG=en_US.UTF-8
	unset LANGUAGE
	make -C modules/mali LICHEE_MOD_DIR=${LICHEE_MOD_DIR} LICHEE_KDIR=${LICHEE_KDIR} \
		install
	)

	#build swl-n20 sdio wifi module
	make -C modules/wifi/nano-c047.12 LICHEE_MOD_DIR=${LICHEE_MOD_DIR} KERNEL_DIR=${LICHEE_KDIR} \
		HOST=${CROSS_COMPILE} INSTALL_DIR=${LICHEE_MOD_DIR} all install

	#build ar6302 sdio wifi module
	make -C modules/wifi/ar6302/AR6K_SDK_ISC.build_3.1_RC.329/host CROSS_COMPILE=${CROSS_COMPILE} \
	        ARCH=arm KERNEL_DIR=${LICHEE_KDIR} INSTALL_DIR=${LICHEE_MOD_DIR} \
	        all install

	#build ar6003 sdio wifi module
	make -C modules/wifi/ar6003/AR6kSDK.build_3.1_RC.514/host CROSS_COMPILE=${CROSS_COMPILE} \
	        ARCH=arm KERNEL_DIR=${LICHEE_KDIR} INSTALL_DIR=${LICHEE_MOD_DIR} \
	        all
}

clean_kernel()
{
	make clean
	rm -rf output/*
}

clean_modules()
{
	echo "Cleaning modules"
	make -C modules/example LICHEE_MOD_DIR=${LICHEE_MOD_DIR} LICHEE_KDIR=${LICHEE_KDIR} clean

	(
	export LANG=en_US.UTF-8
	unset LANGUAGE
	make -C modules/mali LICHEE_MOD_DIR=${LICHEE_MOD_DIR} LICHEE_KDIR=${LICHEE_KDIR} clean
	)

	#build swl-n20 sdio wifi module
	make -C modules/wifi/nano-c047.12 LICHEE_MOD_DIR=${LICHEE_MOD_DIR} KERNEL_DIR=${LICHEE_KDIR} \
		HOST=${CROSS_COMPILE} INSTALL_DIR=${LICHEE_MOD_DIR} clean

	#build ar6302 sdio wifi module
	make -C modules/wifi/ar6302/AR6K_SDK_ISC.build_3.1_RC.329/host CROSS_COMPILE=${CROSS_COMPILE} \
	        ARCH=arm KERNEL_DIR=${LICHEE_KDIR} INSTALL_DIR=${LICHEE_MOD_DIR} \
	        clean
	#build ar6003 sdio wifi module
	make -C modules/wifi/ar6003/AR6kSDK.build_3.1_RC.514/host CROSS_COMPILE=${CROSS_COMPILE} \
	        ARCH=arm KERNEL_DIR=${LICHEE_KDIR} INSTALL_DIR=${LICHEE_MOD_DIR} \
	        clean
}

#####################################################################
#
#                      Main Runtine
#
#####################################################################

LICHEE_ROOT=`(cd ${LICHEE_KDIR}/..; pwd)`
export PATH=${LICHEE_ROOT}/buildroot/output/external-toolchain/bin:$PATH

case "$1" in
kernel)
	build_kernel
	;;
modules)
	build_modules
	;;
clean)
	clean_kernel
	clean_modules
	;;
all)
	build_kernel
	build_modules
	;;
*)
	show_help
	;;
esac

