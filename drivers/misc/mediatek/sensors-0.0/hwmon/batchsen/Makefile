ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-0.0/hwmon/include
ifeq ($(CONFIG_MTK_SCP_SENSORHUB_V1),y)
obj-y := batch_v1.o
else
obj-y := batch.o
endif
