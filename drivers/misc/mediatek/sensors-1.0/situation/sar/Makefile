ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/situation/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/sensorHub/inc_v1
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(CONFIG_MTK_PLATFORM)
obj-y    +=  sarhub.o sar_factory.o
obj-$(CONFIG_SAR_SX933X)    +=  sx933x/
obj-$(CONFIG_SAR_AW9610X)   +=  aw9610x/
obj-$(CONFIG_SAR_AT96T375)   +=  at96t375/
