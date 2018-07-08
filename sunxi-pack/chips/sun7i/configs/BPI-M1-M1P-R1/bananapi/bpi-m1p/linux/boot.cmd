setenv device mmc
setenv partition 0:1
setenv bpi bananapi
setenv board bpi-m1p
setenv chip a20
setenv service linux
#setenv kernel uImage
setenv kernel uImage.sun7i
setenv script script.bin
setenv verbosity 8
#
setenv bootargs "console=ttyS0,115200 console=tty1 board=${board} root=/dev/mmcblk0p2 rootwait rootfstype=ext4 cgroup_enable=memory swapaccount=1 hdmi.audio=EDID:0 disp.screen0_output_mode=1280x720p60 panic=10 consoleblank=0 enforcing=0 loglevel=${verbosity}"
#
# load script.bin
fatload $device ${partition} 0x43000000 ${bpi}/${board}/${service}/${script} || fatload $device ${partition} 0x43000000 script.bin 
# load kernel
#
fatload $device ${partition} 0x48000000 ${bpi}/chip/${chip}/${kernel} || fatload $device ${partition} 0x48000000 uImage
#
# load initramdisk
if fatload $device ${partition} 0x43100000 ${bpi}/uInitrd
then
bootm 0x48000000 0x43100000
else
bootm 0x48000000
fi
#
# convert zImage to uImage:
# mkimage -A arm -O linux -T kernel -C none -a "0x40008000" -e "0x40008000" -n "Linux kernel" -d zImage uImage
#
# Recompile with:
# mkimage -C none -A arm -T script -d boot.cmd boot.scr 
