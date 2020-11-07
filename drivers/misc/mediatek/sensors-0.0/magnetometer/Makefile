# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-0.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-0.0/accelerometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-0.0/magnetometer/inc
ifeq ($(CONFIG_CUSTOM_KERNEL_MAGNETOMETER_MODULE),y)
obj-y   +=  mag_module.o
else
obj-y   +=  mag.o
endif
obj-y   +=  mag_factory.o

obj-$(CONFIG_MTK_K303B_MAG)	+= k303b-mag/
