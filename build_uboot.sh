#!/bin/bash
TOPDIR=`pwd`
export PATH=$TOPDIR/allwinner-tools/gcc-linaro-7.1.1-2017.08-x86_64_arm-linux-gnueabihf/bin:$PATH
cd u-boot-sunxi
./bpi-m1.sh legacy
./bpi-m1p.sh legacy
./bpi-r1.sh legacy
