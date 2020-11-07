# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-0.0/hwmon/include
ifeq ($(CONFIG_CUSTOM_KERNEL_ALSPS_MODULE),y)
obj-y   +=  alsps_module.o
else
obj-y   +=  alsps.o
endif
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

obj-$(CONFIG_MTK_APDS9930)	+=  APDS9930/
obj-$(CONFIG_MTK_APDS9930_NEW)	+=  APDS9930-new/
