#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100_LV7)		+= touch_td4100.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100_LV7)		+= touch_td4100_prd.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100_LV7)		+= touch_td4100_fw_upgrade.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100_LV7)		+= touch_td4100_rmi_dev.o
