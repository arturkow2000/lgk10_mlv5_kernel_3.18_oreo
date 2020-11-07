ifeq ($(CONFIG_LGE_LCM_SETTING),y)
ccflags-y := -Wno-format -Wno-maybe-uninitialized
#ccflags-y += -Wno-error
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/                     \
             -I$(srctree)/drivers/misc/mediatek/video/common/                      \
             -I$(srctree)/drivers/misc/mediatek/video/common/rdma10/               \
             -I$(srctree)/drivers/misc/mediatek/video/common/wdma10/               \
             -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/             \
             -I$(srctree)/drivers/misc/mediatek/sync/                              \
             -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)                \
             -I$(srctree)/drivers/misc/mediatek/cmdq/v2/                           \
             -I$(srctree)/drivers/misc/mediatek/cmdq/v2/$(MTK_PLATFORM)/           \
             -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/mach/ \
             -I$(srctree)/drivers/staging/android/ion/                             \
             -I$(srctree)/drivers/staging/android/ion/mtk/                         \
             -I$(srctree)/drivers/misc/mediatek/ext_disp/                          \
		     -I$(srctree)/drivers/misc/mediatek/ext_disp/v1/                       \
             -I$(srctree)/drivers/misc/mediatek/ext_disp/$(MTK_PLATFORM)/          \
             -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)/        \
             -I$(srctree)/drivers/misc/mediatek/base/power/include/                \
             -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/               \
             -I$(srctree)/drivers/misc/mediatek/include/mt-plat                    \
             -I$(srctree)/drivers/misc/mediatek/mmp/

ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc/

ifeq ($(CONFIG_LGE_LCM_SETTING),y)
obj-y += lge_tuning.o
endif
