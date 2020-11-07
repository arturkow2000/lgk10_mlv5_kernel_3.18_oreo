MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat

# Amp
snd-soc-tpa2015d-objs := tpa2015d.o

obj-$(CONFIG_SND_SOC_TPA2015D)  += snd-soc-tpa2015d.o
obj-$(CONFIG_MTK_LGE_EXT_SPEAKER_AMP) += AudDrv_Ext_Amp.o
obj-$(CONFIG_MTK_LGE_EXT_SPEAKER_AMP) += AudDrv_Gpio_Amp.o
