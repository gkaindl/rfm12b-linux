# Makefile for compiling Kernel
# modules on the fly.

obj-m += rfm12b.o

KVERSION := $(shell uname -r)

# 3.7 moved version.h to a different location
if [ -f /lib/modules/$(KVERSION)/build/include/generated/uapi/linux/version.h ]; then \
	INCLUDE += -I/lib/modules/$(KVERSION)/build/include/generated/uapi/

all:
	make -C /lib/modules/$(KVERSION)/build $(INCLUDE) M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build $(INCLUDE) M=$(PWD) clean

