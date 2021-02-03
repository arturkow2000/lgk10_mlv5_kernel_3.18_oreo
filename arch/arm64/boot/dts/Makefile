dtb-$(CONFIG_ARCH_THUNDER) += thunder-88xx.dtb
dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb

targets += dtbs

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

ifeq ($(strip $(CONFIG_BUILD_ARM64_DTB_OVERLAY_IMAGE)), y)
ifeq ($(CONFIG_MACH_LGE), y)
$(info overlay dtb images build on CONFIG_MACH_LGE enable)
boot := arch/arm64/boot
DTS_BASE_NAME := $(subst $\",,$(CONFIG_BUILD_ARM64_DTB_OVERLAY_IMAGE_NAMES))
LGE_DTS_FILES := $(notdir $(wildcard $(srctree)/$(boot)/dts/lge/$(LGE_TARGET_PLATFORM)-$(DTS_BASE_NAME)/$(LGE_TARGET_PLATFORM)-$(DTS_BASE_NAME)*.dts))
LGE_DTB_DIR := lge/$(LGE_TARGET_PLATFORM)-$(DTS_BASE_NAME)/
LGE_DTB_FILES := $(patsubst %.dts,%.dtb,$(LGE_DTS_FILES))
DTB_LIST += $(addprefix $(LGE_DTB_DIR), $(LGE_DTB_FILES))
else
DTB_LIST += $(addsuffix .dtb,$(subst $\",,$(CONFIG_BUILD_ARM64_DTB_OVERLAY_IMAGE_NAMES)))
endif
endif

targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := dts/*.dtb *.dtb
