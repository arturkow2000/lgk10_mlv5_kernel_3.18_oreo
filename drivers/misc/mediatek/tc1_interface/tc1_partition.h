/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __TC1_PARTITION_H
#define __TC1_PARTITION_H

#include "lg_partition.h"

#define TC1_GET_NAME(fname)	("LGE_"#fname)
#define TC1_FAC_NAME(fname)	LGE_##fname


#define TC1_FAC_IMEI_LEN		LGE_FAC_IMEI_LEN
#define TC1_FAC_NETWORK_CODE_LEN	LGE_FAC_NETWORK_CODE_LEN

#endif
