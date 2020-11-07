custom_cam_cal := $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_CAM_CAL_DRV)))
obj-y += $(custom_cam_cal)

CUSTOM_KERNEL_FLAGS := $(addprefix -D, $(shell echo \
             $(CONFIG_CUSTOM_KERNEL_CAM_CAL_DRV) | tr a-z A-Z))
subdir-ccflags-y += $(CUSTOM_KERNEL_FLAGS)
