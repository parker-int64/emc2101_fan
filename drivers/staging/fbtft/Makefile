SHELL := /bin/sh
obj-m += fb_st7789v.o
KVER ?= $(if $(KERNELRELEASE),$(KERNELRELEASE),$(shell uname -r))
KDIR := /lib/modules/$(KVER)/build

# Install Directory
MODDESTDIR := /lib/modules/$(KVER)/kernel/drivers/staging/fbtft

all:
	@echo "Building fb_st7789v module..."
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	@echo "Cleaning build files..."
	$(MAKE) -C $(KDIR) M=$$PWD clean

install: all
	@echo "Installing fb_st7789v module to $(MODDESTDIR)"
	@install -D -m 644 -t $(MODDESTDIR) *.ko
	@depmod -a

uninstall:
	@echo "Removing fb_st7789v module from $(MODDESTDIR)"
	@modprobe -r -q fb_st7789v
	@rm -vf $(MODDESTDIR)/fb_st7789v.ko
	sudo depmod -a

.PHONY: all clean install uninstall