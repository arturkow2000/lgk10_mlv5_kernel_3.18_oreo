#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
ifeq ($(CONFIG_LGE_ACCDET_IMPEDANCE), y)
subdir-ccflags-y += -Werror -I$(srctree)/sound/soc/mediatek/mt6750/
endif

ifeq ($(CONFIG_LGE_MOISTURE_DETECTION), y)
subdir-ccflags-y += -Werror -I$(srctree)/sound/soc/mediatek/mt6750/
endif

obj-$(CONFIG_MTK_ACCDET) := accdet.o

obj-$(CONFIG_TS3A225E_ACCDET) += ts3a225e.o
