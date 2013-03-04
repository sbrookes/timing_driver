# Makefile for pedagogical driver
# Scott Brookes 1.11.13

# special compilation for a kernel module
obj-m := timing.o

KVERSION := $(shell uname -r)

all:
	make -C /lib/modules/$(KVERSION)/build M=`pwd` modules
#clean:
#	make -C /lib/modules/$(KVERSION)/build M=`pwd` clean

clean:
	rm -f *~
	rm -f *.o
	rm -f *.ko
	rm -f modules.order
	rm -f Module.symvers
	rm -f *#
	rm -f *.mod.c