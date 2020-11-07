#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/ 

obj-$(CONFIG_LGE_TOUCH_SENTRON_SSD6600)      += touch_ssd6600.o
obj-$(CONFIG_LGE_TOUCH_SENTRON_SSD6600)      += touch_ssd6600_fw_upgrade.o

