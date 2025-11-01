SHELL := /bin/sh
obj-m += emc2101.o
KVER ?= $(if $(KERNELRELEASE),$(KERNELRELEASE),$(shell uname -r))
KDIR := /lib/modules/$(KVER)/build

# Install Directory
MODDESTDIR := /lib/modules/$(KVER)/kernel/drivers/hwmon

all:
        @echo "Building emc2101 module..."
        $(MAKE) -C $(KDIR) M=$$PWD modules

clean:
        @echo "Cleaning build files..."
        $(MAKE) -C $(KDIR) M=$$PWD clean

install: all
        @echo "Installing emc2101 module to $(MODDESTDIR)"
        @install -D -m 644 -t $(MODDESTDIR) *.ko
        @depmod -a

uninstall:
        @echo "Removing emc2101 module from $(MODDESTDIR)"
        @rmmod -s emc2101.ko
        @rm -vf $(MODDESTDIR)/emc2101.ko
        sudo depmod -a

.PHONY: all clean install uninstall load unload reload status