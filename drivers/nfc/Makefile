#
# Makefile for nfc devices
#
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat

obj-$(CONFIG_LGE_NFC_PN547_C2)	:= pn547_lge.o
obj-$(CONFIG_NFC_BCM2079X)	+= bcm2079x.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
