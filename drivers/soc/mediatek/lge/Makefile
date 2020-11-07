MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/


obj-$(CONFIG_MACH_LGE)				+= devices_lge.o
obj-$(CONFIG_LGE_BOOTLOADER_LOG)	+= lge_bootloader_log.o
obj-$(CONFIG_LGE_PM_BATTERY_ID_CHECKER) += lge_battery_id.o
obj-$(CONFIG_LGE_BOOT_MODE) += lge_boot_mode.o
obj-$(CONFIG_LGE_HANDLE_PANIC)		+= lge_handle_panic.o
obj-$(CONFIG_LGE_PM_USB_ID)		+= lge_cable_id.o
obj-$(CONFIG_LGE_PM_CHARGER_SCENARIO_COMMON) += lge_charging_scenario.o
obj-$(CONFIG_LGE_PM_CHARGER_SCENARIO_SPRINT) += lge_charging_scenario_sprint.o
obj-$(CONFIG_PRE_SELF_DIAGNOSIS) += lge_pre_selfd.o
obj-$(CONFIG_MACH_LGE)				+= lge_power_sysfs.o
obj-$(CONFIG_LGE_EARJACK_DEBUGGER)	+= lge_earjack_debugger.o
obj-$(CONFIG_LGE_QFPROM_INTERFACE) += lge_efuse_access.o
obj-$(CONFIG_LGE_PM_PSEUDO_BATTERY) += lge_pseudo_batt.o
obj-$(CONFIG_LGE_USB_EMBEDDED_BATTERY) += lge_atd_mid.o

ifeq ($(TARGET_BUILD_VARIANT),user)
ccflags-y += -DUSER_BUILD
endif
