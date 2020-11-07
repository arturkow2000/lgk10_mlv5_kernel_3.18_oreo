#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_SHARP_LR388K6)	+= touch_lr388k6.o
obj-$(CONFIG_LGE_TOUCH_SHARP_LR388K6)	+= touch_lr388k6_fw_upgrade.o
obj-$(CONFIG_LGE_TOUCH_SHARP_LR388K6)	+= touch_lr388k6_prd.o
