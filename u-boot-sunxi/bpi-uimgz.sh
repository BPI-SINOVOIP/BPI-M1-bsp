#!/bin/bash
U=u-boot-2018.01

BOARD=$1
if [ -z $BOARD ] ; then
	echo "usage: $0 BOARD"
	exit 1
fi

echo ${BOARD}
UBOOTBIN=out/${BOARD}/u-boot-sunxi-with-spl.bin
UBOOTIMG=out/${U}-${BOARD}-8k.img
if [ -f ${UBOOTBIN} ] ; then
	cp -a ${UBOOTBIN} ${UBOOTIMG}
else
	UBOOTBIN=out/${BOARD}/u-boot.itb
	SPLBIN=out/${BOARD}/spl/sunxi-spl.bin
	cat ${SPLBIN} ${UBOOTBIN} > ${UBOOTIMG}
fi
rm -f ${UBOOTIMG}.gz
gzip $UBOOTIMG
