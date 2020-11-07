/* touch_nt11206_sd.c
 *
 * Copyright (C) 2015 LGE.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#include <touch_hwif.h>
#include <touch_core.h>
#include "touch_nt11206.h"

#define PSConfig_Tolerance_Negative 120
#define PSConfig_Tolerance_Postive 120
#define PSConfig_TPSelectNegativePercentage 200
#define PSConfig_TPSelectPositivePercentage 200
#define PSConfig_DiffLimitG_Negative -120
#define PSConfig_DiffLimitG_Postive 120

long int boundary[20*30] = {
	1925, 1928, 1899, 1889, 1900, 1889, 1883, 1893, 1900, 1892, 1840, 1862, 1844, 1842, 1867, 1874, 1880, 1905, 1926, 1939, 
	 818,  835,  827,  830,  833,  832,  829,  839,  847,  844,  793,  810,  798,  806,  812,  822,  820,  835,  847,  845,  
	1194, 1216, 1213, 1218, 1220, 1220, 1219, 1218, 1231, 1228, 1216, 1216, 1207, 1215, 1210, 1220, 1215, 1215, 1223, 1213, 
	1263, 1261, 1254, 1255, 1254, 1250, 1250, 1247, 1257, 1256, 1251, 1252, 1241, 1249, 1243, 1259, 1254, 1256, 1268, 1271, 
	 517,  524,  522,  525,  526,  525,  525,  530,  534,  531,  507,  516,  509,  514,  517,  528,  526,  532,  539,  538,  
	 385,  395,  391,  394,  400,  399,  397,  401,  405,  401,  369,  378,  374,  376,  388,  393,  393,  400,  405,  404,  
	 376,  385,  381,  380,  386,  384,  384,  386,  387,  381,  370,  378,  374,  375,  389,  391,  388,  393,  390,  390,  
	 543,  556,  546,  547,  548,  548,  549,  552,  556,  552,  538,  549,  540,  542,  548,  557,  556,  560,  562,  561,  
	1213, 1217, 1211, 1213, 1211, 1209, 1208, 1206, 1217, 1216, 1207, 1215, 1206, 1212, 1205, 1224, 1215, 1219, 1227, 1227, 
	1204, 1207, 1196, 1197, 1197, 1193, 1194, 1192, 1203, 1203, 1196, 1199, 1189, 1195, 1193, 1209, 1204, 1205, 1212, 1214, 
	 533,  544,  536,  537,  537,  538,  537,  542,  545,  542,  526,  535,  530,  531,  542,  548,  547,  553,  550,  552,  
	 363,  368,  364,  364,  368,  367,  366,  369,  370,  365,  353,  360,  358,  359,  374,  375,  374,  378,  374,  376,  
	 355,  356,  352,  351,  355,  355,  353,  356,  357,  353,  344,  349,  346,  350,  361,  363,  361,  363,  360,  360,  
	 511,  520,  513,  514,  515,  515,  514,  519,  522,  518,  504,  514,  508,  511,  522,  525,  523,  528,  526,  525,  
	1102, 1113, 1101, 1102, 1103, 1101, 1099, 1101, 1111, 1111, 1098, 1107, 1096, 1102, 1105, 1115, 1108, 1111, 1115, 1119, 
	1055, 1056, 1048, 1050, 1050, 1048, 1049, 1048, 1058, 1058, 1049, 1052, 1042, 1048, 1051, 1061, 1056, 1056, 1062, 1068, 
	 484,  490,  483,  485,  485,  486,  484,  489,  492,  489,  476,  483,  477,  483,  489,  492,  491,  495,  495,  498,  
	 340,  342,  339,  339,  343,  342,  340,  343,  343,  340,  330,  335,  333,  340,  346,  347,  345,  345,  346,  347,  
	 301,  304,  302,  304,  308,  309,  308,  310,  311,  309,  297,  302,  299,  307,  311,  312,  311,  313,  315,  313,  
	 437,  446,  440,  444,  446,  448,  447,  452,  454,  453,  436,  444,  438,  447,  449,  452,  451,  456,  459,  456,  
	 923,  932,  924,  928,  929,  928,  927,  929,  939,  941,  923,  929,  921,  929,  928,  938,  931,  936,  945,  944,  
	 896,  893,  889,  891,  890,  891,  891,  895,  904,  906,  897,  897,  891,  900,  897,  905,  897,  903,  907,  911,  
	 432,  437,  431,  433,  434,  437,  438,  444,  446,  444,  432,  437,  431,  440,  441,  444,  443,  447,  449,  448,  
	 307,  308,  306,  306,  310,  313,  314,  316,  317,  315,  305,  309,  307,  313,  317,  318,  317,  319,  320,  318,  
	 293,  295,  291,  291,  294,  295,  295,  300,  301,  299,  293,  296,  299,  300,  304,  304,  303,  306,  307,  305,  
	 407,  413,  407,  407,  408,  409,  410,  415,  420,  419,  407,  414,  411,  416,  418,  419,  420,  423,  426,  425,  
	 778,  781,  775,  776,  778,  776,  775,  780,  789,  793,  780,  785,  778,  788,  784,  792,  785,  791,  798,  800,  
	 766,  772,  765,  769,  768,  776,  763,  773,  779,  797,  800,  795,  790,  799,  797,  807,  794,  803,  807,  808,  
	 413,  416,  416,  412,  419,  418,  419,  420,  428,  429,  421,  422,  423,  425,  429,  430,  428,  433,  434,  430,  
	 312,  321,  315,  318,  318,  325,  321,  329,  327,  335,  323,  323,  327,  326,  331,  330,  328,  331,  330,  330,  
};

#define MaxStatisticsBuf 100
static long int golden_Ratio[20*30] = {0, };
static int RawDataTest_Sub(u16 *rawdata, u8 *RecordResult,u8 x_num, u8 y_num);
static int Test_CaluateGRatioAndNormal(u16* rawdata, u8 x_num, u8 y_num);
static int nvt_read_baseline(struct device *dev, u16* xdata)
{
	u8 x_num = 0;
	u8 y_num = 0;

	nvt_change_mode(dev, TEST_MODE_1);

	if(nvt_check_fw_status(dev) != 0)
		return -EAGAIN;

	nvt_get_fw_info(dev);

	nvt_read_mdata(dev, BASELINE_ADDR);
	
	nvt_get_mdata(xdata, &x_num, &y_num);
	//nvt_change_mode(NORMAL_MODE);

	return 0;
}
int nt11206_selftest(struct device *dev, char* buf, u8 mode)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	u16* xdata = NULL;
	u8* record_result = NULL;
	u8 x_num=0;
	u8 y_num=0;
	int sd_ret = 0;
	int ret = 0;
	u8 i = 0;
	u8 j = 0;

	x_num = d->fw.x_axis_num;
	y_num = d->fw.y_axis_num;

	if(	d->resume_state == 0 && !mode) {
		TOUCH_E("LCD OFF, mode:%d\n", mode);
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LCD OFF\n");
		return ret;
	}

	xdata = (u16*)kmalloc(2 * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		sd_ret = -1;
		TOUCH_E("rawdata Alloc Failed\n");
		return sd_ret;
	}
	record_result = (u8*)kmalloc(d->fw.x_axis_num * d->fw.y_axis_num, GFP_KERNEL);
	if(record_result == NULL) {
		sd_ret = -1;
		TOUCH_E("record_result Alloc Failed\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "record_result Alloc Failed\n");
		return ret;
	}
	memset(xdata, 0, 2048 * 2);
	memset(record_result, 0, d->fw.x_axis_num * d->fw.y_axis_num);

	nt11206_hw_reset(dev);
	msleep(500);

	if(nvt_read_baseline(dev, xdata) != 0) {
		sd_ret = 1;
	}
	else {
		//---Self Test Check ---
    	sd_ret = RawDataTest_Sub(xdata, record_result, x_num, y_num);	// 0:PASS, -1:FAIL
	}

	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Channel Status : Pass\n");
	switch(sd_ret) {
		case 0:
			TOUCH_I("Self Test PASS!\n");
			if(!mode) {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Pass\n");
			}
			else {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Pass\n");
			}
			TOUCH_I("RecordResult:\n");
		    for(i=0; i<y_num; i++)
		    {
		        for(j=0; j<x_num; j++)
		        {
		            printk("0x%02X, ", record_result[i*x_num+j]);
					ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "0x%02X, ", record_result[i*x_num+j]);
		        }
				printk("\n");
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
		    }
			TOUCH_I("ReadData:\n");
		    for(i=0; i<y_num; i++)
		    {
		        for(j=0; j<x_num; j++)
		        {
		            printk("%5d ", xdata[i*x_num+j]);
		        }
				printk("\n");
		    }
			break;
			
		case 1:
			TOUCH_E("Self Test ERROR! Read Data FAIL!\n");
			if(!mode) {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Fail\n");
			}
			else {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Fail\n");
			}
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Self Test ERROR! Read Data FAIL!\n");
			break;

		case -1:
			TOUCH_E("Self Test FAIL!\n");
			if(!mode) {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Fail\n");
			}
			else {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Fail\n");
			}
			TOUCH_I("RecordResult:\n");
		    for(i=0; i<y_num; i++)
		    {
		        for(j=0; j<x_num; j++)
		        {
		            printk("0x%02X, ", record_result[i*x_num+j]);
					ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "0x%02X, ", record_result[i*x_num+j]);
		        }
				printk("\n");
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
		    }
			TOUCH_I("ReadData:\n");
		    for(i=0; i<y_num; i++)
		    {
		        for(j=0; j<x_num; j++)
		        {
		            printk("%5d ", xdata[i*x_num+j]);
		        }
				printk("\n");
		    }
			break;
	}

	//---Reset IC---
	nt11206_hw_reset(dev);

	if(xdata)
		kfree(xdata);

	if(record_result)
		kfree(record_result);

	return ret;
}

static int Test_CaluateGRatioAndNormal(u16* rawdata, u8 x_num, u8 y_num)
{
	int i, j, k;
	long int tmpValue;
	long int MaxSum=0;
	int MaxNum=0, MaxIndex=0;
	int Max = -99999; 
	int Min =  99999;
	int offset;	
	int Data;	// double
	int StatisticsStep=0;
	long int StatisticsNum[MaxStatisticsBuf];
	long int StatisticsSum[MaxStatisticsBuf];
    
	//--------------------------------------------------
	//1. (Testing_CM - Golden_CM ) / Testing_CM
	//--------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			Data = rawdata[j*x_num + i];
			if(Data == 0)
				Data = 1;

			golden_Ratio[j*x_num + i] = Data - boundary[j*x_num + i];
			golden_Ratio[j*x_num + i] = ((golden_Ratio[j*x_num + i]*1000) / Data);	// *1000 before division
		}
	}
    
	//--------------------------------------------------------
	// 2. Mutual_GoldenRatio*1000
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			golden_Ratio[j*x_num + i] *= 1000;
		}
	}
    
	//--------------------------------------------------------
	// 3. Calculate StatisticsStep
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			if (Max < golden_Ratio[j*x_num + i])
				Max = (int)golden_Ratio[j*x_num + i];
			if (Min > golden_Ratio[j*x_num + i])
				Min = (int)golden_Ratio[j*x_num + i];
		}
	}

	offset = 0;
	if(Min < 0) // add offset to get erery element Positive
	{
		offset = 0 - Min;	
		for(j=0; j<y_num; j++)
		{
			for(i=0; i<x_num; i++)
			{
				golden_Ratio[j*x_num + i] += offset;
			}
		}
		Max += offset;
	}
	StatisticsStep = Max / MaxStatisticsBuf;
	StatisticsStep += 1;
	if(StatisticsStep < 0)
	{
		TOUCH_E("FAIL! (StatisticsStep < 0)\n");
		return 1;
	}
	
	//--------------------------------------------------------
	// 4. Start Statistics and Average
	//--------------------------------------------------------
	memset(StatisticsSum, 0, sizeof(long int)*MaxStatisticsBuf);
	memset(StatisticsNum, 0, sizeof(int)* MaxStatisticsBuf);
	for(i=0; i<MaxStatisticsBuf; i++)
	{
		StatisticsSum[i] = 0;
		StatisticsNum[i] = 0;
	}
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			tmpValue = golden_Ratio[j*x_num + i];
			tmpValue /= StatisticsStep;
			StatisticsNum[tmpValue] += 2;
			StatisticsSum[tmpValue] += (2*golden_Ratio[j*x_num + i]);

			if((tmpValue + 1) < MaxStatisticsBuf)
			{
				StatisticsNum[tmpValue+1] += 1;
				StatisticsSum[tmpValue + 1] += golden_Ratio[j*x_num + i];
			}
			//if((tmpValue - 1) >= 1)
			//{
				if ((tmpValue - 1) < MaxStatisticsBuf)
				{
					StatisticsNum[tmpValue - 1] += 1;
					StatisticsSum[tmpValue - 1] += golden_Ratio[j*x_num + i];
				}
			//}
			//else
			//{
			//	mPSConfig.myLayerPrint(StrMessageLevel.WarningMsg, "Statistics Index = " + (tmpValue - 1).ToString()); 
			//}
		}
	}
	//Find out Max Statistics
	MaxNum =0;
	for(k=0; k<MaxStatisticsBuf; k++)
	{
		if(MaxNum < StatisticsNum[k])
		{
			MaxSum = StatisticsSum[k];
			MaxNum = StatisticsNum[k];	
			MaxIndex = k;
		}			
	}
	//Caluate Statistics Average
	if(MaxSum > 0)
	{
		tmpValue = (StatisticsSum[MaxIndex] / StatisticsNum[MaxIndex])*2;
		if((MaxIndex+1) < (MaxStatisticsBuf)) 
			tmpValue += (StatisticsSum[MaxIndex+1] / StatisticsNum[MaxIndex+1]); 
		if((MaxIndex-1) >= 0 )
			tmpValue += (StatisticsSum[MaxIndex-1] / StatisticsNum[MaxIndex-1]);

		if((MaxIndex+1) < (MaxStatisticsBuf) &&( (MaxIndex-1) >=0)) 
			tmpValue /=4;
		else
			tmpValue /=3;
	}
	else // Too Separately
	{
		StatisticsSum[0] = 0;
		StatisticsNum[0] = 0;
		for(j=0; j<y_num; j++)
		{
			for(i=0; i<x_num; i++)
			{
				StatisticsSum[0] += (long int)golden_Ratio[j*x_num + i];
				StatisticsNum[0]++;
			}
		}
		tmpValue = StatisticsSum[0] / StatisticsNum[0];
	}
	//----------------------------------------------------------
	//----------------------------------------------------------
	//----------------------------------------------------------
	tmpValue -= offset;
	for(j= 0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			golden_Ratio[j*x_num + i] -= offset;

			golden_Ratio[j*x_num + i] = golden_Ratio[j*x_num + i] - tmpValue;
			golden_Ratio[j*x_num + i] = golden_Ratio[j*x_num + i] / 1000;
		}
	}

	return 0;
}


static int RawDataTest_Sub(u16 *rawdata, u8 *RecordResult,u8 x_num, u8 y_num)
{
	int i, j;
	int kk0=0, kk1=0;
	int kkTPSelectLB = 0;
	int kkTPSelectUB = 0;
	int kkTPSelect = 0;

	//--------------------------------------------------------
	// Init RecordResult array
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for (i=0; i<x_num; i++)
		{
			RecordResult[j*x_num + i] = 0;
		}
	}

	//--------------------------------------------------------
	// 5a. Check abs low boundary
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			kk0 = boundary[j*x_num + i];
			if (kk0 > 0)
			{
				kk0 = (int)((boundary[j*x_num + i]*(1000-PSConfig_Tolerance_Negative)) / 1000);	// *1000 before division
			}
			else
			{
				kk0 = (int)((boundary[j*x_num + i]*(1000+PSConfig_Tolerance_Negative)) / 1000);	// *1000 before division
			}

			if (rawdata[j*x_num + i] < kk0)
			{
				RecordResult[j*x_num + i] |= 0x02;
			}
			else
			{
				kk1 = boundary[j*x_num + i];
				if (kk1 > 0)
				{
					kk1 = (int)((boundary[j*x_num + i]*(1000+PSConfig_Tolerance_Postive)) / 1000);	// *1000 before division
				}
				else
				{
					kk1 = (int)((boundary[j*x_num + i]*(1000-PSConfig_Tolerance_Postive)) / 1000);	// *1000 before division
				}

				if (rawdata[j*x_num + i] > kk1)
				{
					RecordResult[j*x_num + i] |= 0x01;
				}
			}
		}
	}

	//--------------------------------------------------------
	// 5b. Choose the selection TP to verify the rationality of "Golden Sample Boundary"
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			kkTPSelectLB = (int)((boundary[j*x_num + i]*(1000-PSConfig_TPSelectNegativePercentage)) / 1000);	// *1000 before division
			kkTPSelectUB = (int)((boundary[j*x_num + i]*(1000+PSConfig_TPSelectPositivePercentage)) / 1000);	// *1000 before division

			if (rawdata[j*x_num + i] < kkTPSelectLB)
			{
				RecordResult[j*x_num + i] |= 0x04;
			}
			else if (rawdata[j*x_num + i] > kkTPSelectUB)
			{
				RecordResult[j*x_num + i] |= 0x04;
			}
		}
	}

	//--------------------------------------------------
	//6. (Testing_CM - Golden_CM ) / Testing_CM
	//--------------------------------------------------
	Test_CaluateGRatioAndNormal(rawdata, x_num, y_num);

	//--------------------------------------------------------
	// 7 . Check Golden Ratio Test
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			if(golden_Ratio[j*x_num + i] < PSConfig_DiffLimitG_Negative)
			{
				RecordResult[j*x_num + i] |= 0x08;
			}
			if(golden_Ratio[j*x_num + i] > PSConfig_DiffLimitG_Postive)
			{
				RecordResult[j*x_num + i] |= 0x04;
			}
		}
	}

	//--------------------------------------------------------
	// 8 . Record Test Result & Select TP Panel
	//--------------------------------------------------------
	kk0 = 0;
	kkTPSelect = 0;
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			if((RecordResult[j*x_num + i] & 0x01) > 0)
				kk0++;
			if((RecordResult[j*x_num + i] & 0x02) > 0)
				kk0++;
			if((RecordResult[j*x_num + i] & 0x04) > 0)
				kk0++;
			if((RecordResult[j*x_num + i] & 0x08) > 0)
				kk0++;
		}
	}

	if(kk0 >= 1)
	{
		return -1;	// FAIL
	}
	else
	{
		return 0;	// PASS
	}          
}
