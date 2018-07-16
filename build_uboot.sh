#!/bin/bash
TOPDIR=`pwd`
export PATH=$TOPDIR/allwinner-tools/gcc-linaro-7.1.1-2017.08-x86_64_arm-linux-gnueabihf/bin:$PATH
cd u-boot-sunxi

# $1=lcd, build lcd type
# $1 is none, build hdmi type
./bpi-m1.sh legacy $1
./bpi-m1p.sh legacy $1
./bpi-r1.sh legacy $1
