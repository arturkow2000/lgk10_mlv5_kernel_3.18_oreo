#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_MELFAS_MIT300)      += touch_mit300.o
obj-$(CONFIG_LGE_TOUCH_MELFAS_MIT300)      += touch_mit300_fw_upgrade.o
obj-$(CONFIG_LGE_TOUCH_MELFAS_MIT300)      += touch_mit300_prd.o
obj-$(CONFIG_LGE_TOUCH_MELFAS_MIT300)      += touch_mit300_mip_dev.o

