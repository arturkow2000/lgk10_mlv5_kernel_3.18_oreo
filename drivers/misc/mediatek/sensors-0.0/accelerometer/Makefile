# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-0.0/hwmon/include

ifeq ($(CONFIG_CUSTOM_KERNEL_ACCELEROMETER_MODULE),y)
obj-y   +=  accel_module.o accel_factory.o
else
obj-y   +=  accel.o accel_factory.o
endif

obj-$(CONFIG_MTK_K303B_ACC)   +=  k303b-acc/
obj-$(CONFIG_MTK_K303B_ACC_NEW)   +=  k303b-acc-new/
