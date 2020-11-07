#
# Makefile for misc devices that really don't fit anywhere else.
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/chargepump/inc/

obj-$(CONFIG_LGD_INCELL_LG4894_HD_LV5) += lg4894_hd720_dsi_vdo.o
obj-$(CONFIG_LGD_INCELL_LG4894_HD_K6PNA) += lg4894_hd720_dsi_vdo_k6pna.o