PATH = /opt/CodeSourcery/Sourcery_G++_Lite
CC = $(PATH)/bin/arm-none-linux-gnueabi-gcc
CFLAGS = -I$(PATH)/arm-none-linux-gnueabi/libc/usr/include/

SRCS =
SRCS += testusb.c

all:
	$(CC) $(CFLAGS) $(SRCS) -lpthread -o testusb

clean:
	/bin/rm -f testusb
