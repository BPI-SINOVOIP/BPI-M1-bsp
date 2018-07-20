# BPI-M1-bsp
Supports Banana Pi BPI-M1 / BPI-M1+ / BPI-R1 (Kernel3.4)


----------
**Build**

uboot  hdmi type

    $ ./build_uboot.sh

uboot lcd type

    $ ./build_uboot.sh lcd

kernel and pack

    $ ./build.sh BPI-M1-M1P-R1 1


----------


**Install**

	

    $ sudo bpi-bootsel SD/bpi-m1-m1p-r1/100MB/u-boot-2018.01-bpi-${board}-legacy-${type}-8k.img.gz /dev/sdX
    $ cd SD/bpi-m1-m1p-r1
    $ sudo bpi-update -d /dev/sdX
