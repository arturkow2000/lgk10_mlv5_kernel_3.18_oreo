# Copyright (C) 2017 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.

LOCAL_PATH := $(call my-dir)

ifeq ($(notdir $(LOCAL_PATH)),$(strip $(LINUX_KERNEL_VERSION)))
ifneq ($(strip $(TARGET_NO_KERNEL)),true)
include $(LOCAL_PATH)/kenv.mk

TUXERA_MODULES_OUT := $(TARGET_OUT_VENDOR_SHARED_LIBRARIES)/modules

ifeq ($(PRODUCT_SUPPORT_EXFAT), y)
sinclude $(ANDROID_BUILD_TOP)/device/lge/common/tuxera.mk
endif

# set LGE device name
# lv7v project use TARGET_DEVICE as mlv7, but we want to use mlv7v in driver level
ifeq ($(MTK_PROJECT_NAME), muse6750_lv7v_o)
  LGE_TARGET_DEVICE_REAL=mlv7v
else
  LGE_TARGET_DEVICE_REAL=$(TARGET_DEVICE)
endif

# Add LGE target feature
KERNEL_MAKE_OPTION := $(KERNEL_MAKE_OPTION) \
LGE_TARGET_PLATFORM=$(TARGET_BOARD_PLATFORM) \
LGE_TARGET_DEVICE=$(LGE_TARGET_DEVICE_REAL)

ifeq ($(strip $(BUILD_USES_BSP_FEATURES_LOG)),true)
FEATURE_BUILD_LOG_OUT := BSP_features_$(TARGET_BUILD_VARIANT)_for_$(TARGET_PRODUCT)
endif

#change lz4 tool from lz4c to lz4demo for compatibilty with fota tool(lz4elix)
ifeq ($(strip $(TARGET_KERNEL_COMPRESSION_TOOL)), lz4demo)
    KERNEL_MAKE_OPTION := $(KERNEL_MAKE_OPTION) TARGET_KERNEL_COMPRESSION_TOOL=lz4demo
    LZ4DEMO := $(HOST_OUT_EXECUTABLES)/lz4demo$(HOST_EXECUTABLE_SUFFIX)
else
    LZ4DEMO :=
endif

ifneq (yes,$(filter $(MTK_BSP_PACKAGE) $(MTK_BASIC_PACKAGE),yes))
ifneq ($(strip $(MTK_EMMC_SUPPORT)),yes)
ifeq  ($(strip $(MTK_NAND_UBIFS_SUPPORT)),yes)
    KERNEL_MAKE_OPTION += LOCALVERSION=
endif
endif
endif

ifeq ($(wildcard $(TARGET_PREBUILT_KERNEL)),)
# .config cannot be PHONY due to config_data.gz
$(TARGET_KERNEL_CONFIG): $(KERNEL_CONFIG_FILE) $(LOCAL_PATH)/Android.mk
$(TARGET_KERNEL_CONFIG): $(shell find $(KERNEL_DIR) -name "Kconfig*") | $(KERNEL_OUT)
	echo "Generating a .config file for header generation"
	$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) $(KERNEL_DEFCONFIG)
ifeq ($(strip $(BUILD_USES_BSP_FEATURES_LOG)),true)
	$(hide) mkdir -p $(PRODUCT_OUT)/$(FEATURE_BUILD_LOG_OUT)
	$(hide) rm -f $(PRODUCT_OUT)/$(FEATURE_BUILD_LOG_OUT)/kernel_config
	cat $(KERNEL_OUT)/.config | grep -v "#" | grep -v '^$$' > $(PRODUCT_OUT)/$(FEATURE_BUILD_LOG_OUT)/kernel_config
endif

$(KERNEL_MODULES_DEPS): $(KERNEL_ZIMAGE_OUT) ;
$(BUILT_DTB_OVERLAY_TARGET): $(KERNEL_ZIMAGE_OUT)

.KATI_RESTAT: $(KERNEL_ZIMAGE_OUT)
$(KERNEL_ZIMAGE_OUT): $(TARGET_KERNEL_CONFIG) FORCE | $(KERNEL_OUT) $(LZ4DEMO)
	$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION)
	$(hide) $(call fixup-kernel-cmd-file,$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/compressed/.piggy.xzkern.cmd)
ifneq ($(KERNEL_CONFIG_MODULES),)
	#$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) INSTALL_MOD_PATH=$(KERNEL_MODULES_SYMBOLS_OUT) modules_install
	#$(hide) $(call move-kernel-module-files,$(KERNEL_MODULES_SYMBOLS_OUT),$(KERNEL_OUT))
	#$(hide) $(call clean-kernel-module-dirs,$(KERNEL_MODULES_SYMBOLS_OUT),$(KERNEL_OUT))
	#$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT) modules_install
	#$(hide) $(call move-kernel-module-files,$(KERNEL_MODULES_OUT),$(KERNEL_OUT))
	#$(hide) $(call clean-kernel-module-dirs,$(KERNEL_MODULES_OUT),$(KERNEL_OUT))
endif

ifeq ($(USE_LGE_VPN), true)
	@mkdir -p $(ANDROID_BUILD_TOP)/$(TUXERA_MODULES_OUT)
	@cp $(KERNEL_OUT)/net/netfilter/interceptor_v36/vpnclient.ko $(ANDROID_BUILD_TOP)/$(TARGET_OUT_VENDOR_SHARED_LIBRARIES)/modules/
	@$(ANDROID_BUILD_TOP)/kernel-3.18/scripts/sign-file sha1 $(ANDROID_BUILD_TOP)/$(KERNEL_OUT)/signing_key.priv $(ANDROID_BUILD_TOP)/$(KERNEL_OUT)/signing_key.x509 $(TARGET_OUT_VENDOR_SHARED_LIBRARIES)/modules/vpnclient.ko
endif

ifeq ($(PRODUCT_SUPPORT_EXFAT), y)
ifneq ($(SUPPORT_EXFAT_TUXERA), )
	@cp -f $(ANDROID_BUILD_TOP)/kernel-3.18/tuxera_update.sh $(ANDROID_BUILD_TOP)
	@sh tuxera_update.sh --target target/lg.d/mt6750s --use-cache --latest --max-cache-entries 2 --source-dir $(ANDROID_BUILD_TOP)/kernel-3.18 --output-dir $(KERNEL_OUT) $(SUPPORT_EXFAT_TUXERA)
	@tar -xzf tuxera-exfat*.tgz
	@mkdir -p $(ANDROID_BUILD_TOP)/$(TUXERA_MODULES_OUT)
	@mkdir -p $(ANDROID_BUILD_TOP)/$(TARGET_OUT_EXECUTABLES)
	@cp $(ANDROID_BUILD_TOP)/tuxera-exfat*/exfat/kernel-module/texfat.ko $(ANDROID_BUILD_TOP)/$(TUXERA_MODULES_OUT)
	@cp $(ANDROID_BUILD_TOP)/tuxera-exfat*/exfat/tools/* $(ANDROID_BUILD_TOP)/$(TARGET_OUT_EXECUTABLES)
	#perl $(ANDROID_BUILD_TOP)/kernel-3.18/scripts/sign-file sha1 $(KERNEL_OUT)/signing_key.priv $(KERNEL_OUT)/signing_key.x509 $(TUXERA_MODULES_OUT)/texfat.ko
	@$(ANDROID_BUILD_TOP)/kernel-3.18/scripts/sign-file sha1 $(ANDROID_BUILD_TOP)/$(KERNEL_OUT)/signing_key.priv $(ANDROID_BUILD_TOP)/$(KERNEL_OUT)/signing_key.x509 $(TUXERA_MODULES_OUT)/texfat.ko
	@rm -f kheaders*.tar.bz2
	@rm -f tuxera-exfat*.tgz
	@rm -rf tuxera-exfat*
	@rm -f tuxera_update.sh
endif
endif

ifeq ($(strip $(MTK_HEADER_SUPPORT)), yes)
$(BUILT_KERNEL_TARGET): $(KERNEL_ZIMAGE_OUT) $(LOCAL_PATH)/Android.mk $(KERNEL_HEADERS_INSTALL) | $(HOST_OUT_EXECUTABLES)/mkimage$(HOST_EXECUTABLE_SUFFIX)
	$(hide) $(HOST_OUT_EXECUTABLES)/mkimage$(HOST_EXECUTABLE_SUFFIX) $< KERNEL 0xffffffff > $@
else
$(BUILT_KERNEL_TARGET): $(KERNEL_ZIMAGE_OUT) $(LOCAL_PATH)/Android.mk $(KERNEL_HEADERS_INSTALL) | $(ACP)
	$(copy-file-to-target)
endif

$(TARGET_PREBUILT_KERNEL): $(BUILT_KERNEL_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-new-target)

endif#TARGET_PREBUILT_KERNEL is empty

$(INSTALLED_DTB_OVERLAY_TARGET): $(BUILT_DTB_OVERLAY_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-target)

ifeq ($(strip $(MTK_DTBO_FEATURE)), yes)
droid: $(INSTALLED_DTB_OVERLAY_TARGET)
endif

$(INSTALLED_KERNEL_TARGET): $(BUILT_KERNEL_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-target)

ifneq ($(KERNEL_CONFIG_MODULES),)
$(BUILT_SYSTEMIMAGE): $(KERNEL_MODULES_DEPS)
endif

.PHONY: kernel save-kernel kernel-savedefconfig %config-kernel clean-kernel odmdtboimage

KERNEL_HEADER_DEFCONFIG := $(strip $(KERNEL_HEADER_DEFCONFIG))
ifeq ($(KERNEL_HEADER_DEFCONFIG),)
KERNEL_HEADER_DEFCONFIG := $(KERNEL_DEFCONFIG)
endif
KERNEL_CONFIG := $(TARGET_KERNEL_CONFIG)

kernel: $(INSTALLED_KERNEL_TARGET)
save-kernel: $(TARGET_PREBUILT_KERNEL)

kernel-savedefconfig: $(TARGET_KERNEL_CONFIG)
	cp $(TARGET_KERNEL_CONFIG) $(KERNEL_CONFIG_FILE)

kernel-menuconfig: | $(KERNEL_OUT)
	$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) menuconfig

%config-kernel: | $(KERNEL_OUT)
	$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) $(patsubst %config-kernel,%config,$@)

clean-kernel:
	$(hide) rm -rf $(KERNEL_OUT) $(KERNEL_MODULES_OUT) $(INSTALLED_KERNEL_TARGET)
	$(hide) rm -f $(INSTALLED_DTB_OVERLAY_TARGET)

ifeq ($(strip $(MTK_DTBO_FEATURE)), yes)
.PHONY: odmdtboimage
odmdtboimage: $(TARGET_KERNEL_CONFIG)
	$(hide) mkdir -p $(KERNEL_OUT)
	$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) odmdtboimage
	$(hide) cp $(BUILT_DTB_OVERLAY_TARGET) $(INSTALLED_DTB_OVERLAY_TARGET)
endif

$(KERNEL_OUT):
	$(hide) mkdir -p $@

$(KERNEL_HEADERS_TIMESTAMP) : $(KERNEL_HEADERS_INSTALL)
$(KERNEL_HEADERS_INSTALL) : $(TARGET_KERNEL_CONFIG) | $(KERNEL_OUT)
	$(hide) if [ ! -z "$(KERNEL_HEADER_DEFCONFIG)" ]; then \
				rm -f ../$(KERNEL_CONFIG); \
				$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) $(KERNEL_HEADER_DEFCONFIG) headers_install; fi
	$(hide) if [ "$(KERNEL_HEADER_DEFCONFIG)" != "$(KERNEL_DEFCONFIG)" ]; then \
				echo "Used a different defconfig for header generation"; \
				rm -f ../$(KERNEL_CONFIG); \
				$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) $(KERNEL_DEFCONFIG); fi
	$(hide) if [ ! -z "$(KERNEL_CONFIG_OVERRIDE)" ]; then \
				echo "Overriding kernel config with '$(KERNEL_CONFIG_OVERRIDE)'"; \
				echo $(KERNEL_CONFIG_OVERRIDE) >> $(TARGET_KERNEL_CONFIG); \
				$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) oldconfig; fi
	$(hide) touch $@/build-timestamp


.PHONY: check-kernel-config check-kernel-dotconfig
droid: check-kernel-config check-kernel-dotconfig
check-mtk-config: check-kernel-config check-kernel-dotconfig
check-kernel-config: PRIVATE_COMMAND := $(if $(wildcard device/mediatek/build/build/tools/check_kernel_config.py),$(if $(filter yes,$(DISABLE_MTK_CONFIG_CHECK)),-)python device/mediatek/build/build/tools/check_kernel_config.py -c $(MTK_TARGET_PROJECT_FOLDER)/ProjectConfig.mk -k $(KERNEL_CONFIG_FILE) -p $(MTK_PROJECT_NAME))
check-kernel-config:
	$(PRIVATE_COMMAND)

ifneq ($(filter check-mtk-config check-kernel-dotconfig,$(MAKECMDGOALS)),)
.PHONY: $(TARGET_KERNEL_CONFIG)
endif
check-kernel-dotconfig: PRIVATE_COMMAND := $(if $(wildcard device/mediatek/build/build/tools/check_kernel_config.py),$(if $(filter yes,$(DISABLE_MTK_CONFIG_CHECK)),-)python device/mediatek/build/build/tools/check_kernel_config.py -c $(MTK_TARGET_PROJECT_FOLDER)/ProjectConfig.mk -k $(TARGET_KERNEL_CONFIG) -p $(MTK_PROJECT_NAME))
check-kernel-dotconfig: $(TARGET_KERNEL_CONFIG)
	$(PRIVATE_COMMAND)


endif#TARGET_NO_KERNEL
endif#LINUX_KERNEL_VERSION
