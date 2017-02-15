#
# Detect running kernel configuration
#
RELEASE ?= $(shell /bin/uname -r )
#############
# Linux 2.6 #
#############

# Allow environment variables to override submake -C and O= settings
#LINUXSRC_DIR ?= /usr/src/linux
LINUXSRC_DIR ?= /lib/modules/$(RELEASE)/build
OBJBUILD_DIR ?= /lib/modules/$(RELEASE)/build

# Build is actually done in /usr/src/linux-2.6.5-7.97-obj/x86_64/smp

EXTRA_CFLAGS += -Wall

# To enable MSI (Message Signalled Interrupt) support pass csx_use_msi=1 to insmod
# Invoke Linux 2.6 kbuild infrastructure for all targets except "all".
# "all" isn't a recognized kbuild target, so filter it out and just let
# the default build happen if that's the target.
all $(MAKECMDGOALS):
	$(MAKE) -C $(LINUXSRC_DIR) O=$(OBJBUILD_DIR) M=`pwd` \
		$(filter-out all,$(MAKECMDGOALS))

# Add rfm2g to the list of modules to build
obj-m += csx.o

# Specify object files that comprise rfm2g.o
csx-objs := csx_driver.o
