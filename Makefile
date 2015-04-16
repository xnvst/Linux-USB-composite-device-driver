#include ${EZSDK}/Rules.make

ifndef LINUX_MV_NETRA
LINUX_MV_NETRA=/opt/ezsdk505/board-support/linux-2.6.37-psp04.04.00.01
#LINUX_MV_NETRA=../../../../../../../netra-kernel-20130226-1540
endif

#CFLAGS += -Wall -D__KERNEL__ -DMODULE -I$(LINUX_MV_NETRA)/include

PATH := ${PATH}:/opt/CodeSourcery/Sourcery_G++_Lite/bin/
PHONY := all clean
all:
	make -C $(LINUX_MV_NETRA) M=`pwd` ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- modules
#	cp *.ko /home/bliu/workdir/filesys/opt/aco2_im/bin 

clean:
	make -C $(LINUX_MV_NETRA) M=`pwd` ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- clean

obj-m := g_cmd_stat.o
g_cmd_stat-objs := cmd_stat.o file_ops.o traps.o get.o set.o
