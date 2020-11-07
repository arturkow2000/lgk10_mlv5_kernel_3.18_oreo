#
# Makefile for the TDMB FC8050 drivers.
#

# Each configuration option enables a list of files.

ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)

# LGE
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += broadcast_fc8080_drv_if.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += broadcast_fc8080.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += tdmb_tunerbbdrv_fc8080.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += bbm.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fc8080_bb.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fc8080_demux.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fc8080_i2c.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fc8080_isr.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fc8080_ppi.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fc8080_spi.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fc8080_tun.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fci_hal.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fci_oal.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fci_tun.o
obj-$(CONFIG_LGE_BROADCAST_TDMB)        += fci_hpi.o
EXTRA_CFLAGS += -I$(srctree)/drivers/misc/mediatek/broadcast/tdmb/fc8080/inc

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
EXTRA_CFLAGS += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
endif