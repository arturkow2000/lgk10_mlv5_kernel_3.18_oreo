#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include <linux/delay.h>
#include <linux/slab.h>

#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8180_regs.h"
#include "fc8180_isr.h"
#include "fci_hal.h"
#include "fci_types.h"
#include "fc8180_drv_api.h"

#include "broadcast_fc8180.h"
#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"

//#define _DISPLAY_MONITOR_DBG_LOG_
/*#define _USE_ONSEG_SIGINFO_MODIFIED_MODE_*/
#define _USE_MONITORING_TIME_GAP_

enum EnumIsdbType{
	TMM_13SEG = 0,
	TMM_1SEG,
	UHF_1SEG,
	UHF_13SEG,
};

enum EnumSigInfo{
	ENUM_GET_ALL = 1,
	ENUM_GET_BER,
	ENUM_GET_PER,
	ENUM_GET_CN,
	ENUM_GET_CN_PER_LAYER,
	ENUM_GET_LAYER_INFO,
	ENUM_GET_RECEIVE_STATUS,
	ENUM_GET_RSSI,
	ENUM_GET_SCAN_STATUS,
	ENUM_GET_SYS_INFO,
	ENUM_GET_TMCC_INFO,
	ENUM_GET_ONESEG_SIG_INFO
};

static unsigned int frequencyTable[56] = {
	473143, 479143, 485143, 491143, 497143,
	503143, 509143, 515143, 521143, 527143,
	533143, 539143, 545143, 551143, 557143,
	563143, 569143, 575143, 581143, 587143,
	593143, 599143, 605143, 611143, 617143,
	623143, 629143, 635143, 641143, 647143,
	653143, 659143, 665143, 671143, 677143,
	683143, 689143, 695143, 701143, 707143,
	713143, 719143, 725143, 731143, 737143,
	743143, 749143, 755143, 761143, 767143,
	773143, 779143, 785143, 791143, 797143,
	803143,
};

static int currentBroadCast = TMM_13SEG;
static int currentSelectedChannel = -1;

s32 OnAir;
s32 broad_type;

extern void broadcast_fci_ringbuffer_flush(void);

/* Body of Internel function */
int	broadcast_fc8180_drv_start(void)
{
	int rc;
	rc = OK;
	print_log(NULL, "[1seg]broadcast_drv_start\n");
	return rc;
}

int	broadcast_fc8180_get_stop_mode(void)
{
	int rc;
	rc = OK;
	print_log(NULL, "[1seg]broadcast_get_stop_mode\n");
	return rc;
}

int	broadcast_fc8180_drv_if_power_on(void)
{
	int rc = ERROR;

	print_log(NULL, "[1seg][YK_26_crystal] broadcast_drv_if_power_on\n");
	if (!fc8180_is_power_on())
		rc = fc8180_power_on();
	return rc;
}

int	broadcast_fc8180_drv_if_power_off(void)
{
	int rc = ERROR;
	print_log(NULL, "[1seg]broadcast_drv_if_power_off\n");
	if (fc8180_is_power_on())
		rc = fc8180_power_off();
	else
		print_log(NULL, "[1seg] warning-already power off\n");

	return rc;
}

int	broadcast_fc8180_drv_if_open(void)
{
	int ret = 0;
       print_log(NULL, "[1seg][YK][3] broadcast_fc8180_drv_if_open\n");
	broad_type = ISDBT_1_SEG_TYPE;
	ret = tunerbb_drv_fc8180_init(broad_type);

	if (ret)
		return ERROR;
	else {
		OnAir = 1;
		return OK;
	}
}

int	broadcast_fc8180_drv_if_close(void)
{
	int ret = 0;

	fc8180_stop();
	ret = tunerbb_drv_fc8180_stop();
	fci_irq_disable();
	OnAir = 0;

	if (ret)
		return ERROR;
	else
		return OK;
}

unsigned long long fcTimer;
void setTimer(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	fcTimer = (long long) tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

unsigned long long TimeCount_ms(void)
{
	unsigned long long tickcount = 0;
	struct timeval tv;

	do_gettimeofday(&tv);
	tickcount = (long long) tv.tv_sec * 1000 + tv.tv_usec / 1000;
	return tickcount;
}

int	broadcast_fc8180_drv_if_set_channel(struct broadcast_dmb_set_ch_info *udata)
{
	signed long frequency = 214714; /*tmm*/
	int ret;

	fci_irq_disable();

	setTimer();

	if (OnAir == 0 || udata == NULL) {
		print_log(NULL, "[1seg] broadcast_drv_if_set_channel\
			error [!OnAir]\n");
		return ERROR;
	}

	/* uhf 1segment */
	currentSelectedChannel = udata->channel;

	if (udata->segment == 13)
		currentBroadCast = UHF_13SEG;
	else
		currentBroadCast = UHF_1SEG;

	#ifdef CONFIG_LGE_BROADCAST_SBTVD_LATIN
	if (udata->channel < 14 || udata->channel > 69) {
		print_log(NULL, "[1seg] channel information error\n");
		return ERROR;
	}
	frequency = frequencyTable[udata->channel-14];
	#else
	if (udata->channel < 13 || udata->channel > 62) {
		print_log(NULL, "[1seg] channel information error\n");
		return ERROR;
	}
	frequency = frequencyTable[udata->channel-13];
	#endif

	/* Scan mode(udata->mode==1) & need lock check */
	fci_irq_enable();
	ret = tunerbb_drv_fc8180_set_channel(frequency, udata->mode);
	broadcast_fci_ringbuffer_flush();

	if (ret)
		return ERROR;

	print_log(NULL, "[fc8180] channel channel : %d, %d, %d, scan OK\n"
		, udata->channel, udata->mode, udata->subchannel);

	return OK;
}

int	broadcast_fc8180_drv_if_resync(void)
{
	int rc;
	/*
	FC8300 use auto-resync
	*/
	rc = OK;
	return rc;
}

int	broadcast_fc8180_drv_if_detect_sync(struct broadcast_dmb_sync_info *udata)
{
	int sync;
	if (OnAir == 0) {
		print_log(NULL, "[1seg] broadcast_drv_if_detect_sync\
			error [!OnAir]\n");
		return ERROR;
	}

	sync = tunerbb_drv_fc8180_Get_SyncStatus();

	udata->sync_status = sync;
	udata->sync_ext_status = sync;

	return OK;
}

static void broadcast_fc8180_drv_if_get_oneseg_sig_info(struct fc8180Status_t *pst
	, struct broadcast_dmb_control_info *pInfo, s32 brd_type)
{
	int ber;
	int per;
	int cn;

	pInfo->sig_info.info.oneseg_info.lock = pst->lock;
	cn = pInfo->sig_info.info.oneseg_info.cn = pst->cn;
	ber = pInfo->sig_info.info.oneseg_info.ber = pst->ber_1seg;
	per = pInfo->sig_info.info.oneseg_info.per = pst->per_1seg;

	pInfo->sig_info.info.oneseg_info.agc = pst->agc;
	pInfo->sig_info.info.oneseg_info.rssi = pst->rssi / (-100);
	pInfo->sig_info.info.oneseg_info.ErrTSP = pst->ErrTSP;
	pInfo->sig_info.info.oneseg_info.TotalTSP = pst->TotalTSP;

	pInfo->cmd_info.over = pst->agc;
	pInfo->sig_info.info.mmb_info.antenna_level_oneseg
		= pst->antenna_level_oneseg;

	pInfo->sig_info.info.oneseg_info.Num = 0;
	pInfo->sig_info.info.oneseg_info.Exp = 0;
	pInfo->sig_info.info.oneseg_info.mode = 0;


#ifdef _DISPLAY_MONITOR_DBG_LOG_
	print_log(NULL, "[1seg][monitor] lock[%d] Antenna[%d]\
	cn[%d] ber[%d] per[%d] rssi[%d] errTSP[%d/%d]\n",
			pInfo->sig_info.info.oneseg_info.lock,
			pInfo->sig_info.info.mmb_info.antenna_level_oneseg,
			pInfo->sig_info.info.oneseg_info.cn,
			pInfo->sig_info.info.oneseg_info.ber,
			pInfo->sig_info.info.oneseg_info.per,
			pInfo->sig_info.info.oneseg_info.rssi,
			pInfo->sig_info.info.oneseg_info.ErrTSP,
			pInfo->sig_info.info.oneseg_info.TotalTSP);

#endif
}


struct fc8180Status_t st;
int	broadcast_fc8180_drv_if_get_sig_info(struct broadcast_dmb_control_info *pInfo)
{
	int layer;
	mmb_sig_info *fullseg_info;

	//print_log(NULL, "[FC8180] broadcast_drv_if_get_sig_info  %d, %d\n"
	//	, OnAir, pInfo->cmd_info.cmd);
	if (OnAir == 0 || pInfo == NULL)
		return ERROR;

	fullseg_info = &pInfo->sig_info.info.mmb_info;

	layer = pInfo->cmd_info.layer;

	if (pInfo->cmd_info.cmd == ENUM_GET_ALL)
			tunerbb_drv_fc8180_Get_SignalInfo(&st, broad_type);

	switch (pInfo->cmd_info.cmd) {
	case ENUM_GET_ALL:
		fullseg_info->cn = st.cn;

		fullseg_info->ber_a = st.ber_1seg;

		fullseg_info->per_a = st.per_1seg;

		fullseg_info->layerinfo_a = st.layerinfo_a;

		fullseg_info->tmccinfo = 0xFF;

		fullseg_info->receive_status = 0xFF;

		fullseg_info->rssi = st.rssi;

		fullseg_info->scan_status = 0xFF;

		fullseg_info->sysinfo = 0xFF;

		fullseg_info->total_tsp_a = st.total_tsp_1seg;

		fullseg_info->ber_b = 100000;

		fullseg_info->per_b = 100000;

		fullseg_info->layerinfo_b = 0xFFFF;

		fullseg_info->total_tsp_b = 0;

		fullseg_info->ber_c = 100000;

		fullseg_info->per_c = 100000;

		fullseg_info->layerinfo_c = 0xFFFF;

		fullseg_info->total_tsp_c = 0;

		fullseg_info->antenna_level_fullseg = 0;

		fullseg_info->antenna_level_oneseg = st.antenna_level_oneseg;

		fullseg_info->agc = st.agc;

		fullseg_info->ber_1seg = st.ber_1seg;

		fullseg_info->per_1seg = st.per_1seg;

		fullseg_info->total_tsp_1seg = st.total_tsp_1seg;

		fullseg_info->err_tsp_1seg = st.err_tsp_1seg;

		fullseg_info->ber_fullseg = 100000;

		fullseg_info->per_fullseg = 100000;

		fullseg_info->total_tsp_fullseg = 0;

		fullseg_info->err_tsp_fullseg = 0;

	break;

	case ENUM_GET_BER:
		fullseg_info->ber_1seg = st.ber_1seg;
	break;

	case ENUM_GET_PER:
		fullseg_info->per_1seg = st.per_1seg;
	break;

	case ENUM_GET_CN:
		fullseg_info->cn = st.cn;
	break;

	case ENUM_GET_CN_PER_LAYER:
	break;

	case ENUM_GET_LAYER_INFO:
		fullseg_info->layerinfo_a = st.layerinfo_a;
	break;

	case ENUM_GET_RECEIVE_STATUS:
		fullseg_info->receive_status = 0xFF;
	break;

	case ENUM_GET_RSSI:
		fullseg_info->rssi = st.rssi;
	break;

	case ENUM_GET_SCAN_STATUS:
		fullseg_info->scan_status = 0xFF;
	break;

	case ENUM_GET_SYS_INFO:
		fullseg_info->sysinfo = 0xFF;
	break;

	case ENUM_GET_TMCC_INFO:
		fullseg_info->tmccinfo = 0xFF;
	break;

	case ENUM_GET_ONESEG_SIG_INFO:
		broadcast_fc8180_drv_if_get_oneseg_sig_info(&st, pInfo, broad_type);
	break;

	default:
		print_log(NULL, "[mmbi][monitor] sig_info unknown\
			command[%d]\n", pInfo->cmd_info.cmd);
		return -1;
	break;
	}

	return OK;
}

int	broadcast_fc8180_drv_if_get_ch_info(struct broadcast_dmb_ch_info *ch_info)
{
	int rc = ERROR;

	if (OnAir == 0) {
		print_log(NULL, "[1seg][no semaphore]\
			broadcast_drv_if_get_ch_info error [!OnAir]\n");
		return ERROR;
	}

	/*
	Unused function
	*/
	rc = OK;
	return rc;
}

int	broadcast_fc8180_drv_if_get_dmb_data(struct broadcast_dmb_data_info *pdmb_data)
{
	unsigned int total_len;

	if (OnAir == 0 || pdmb_data == NULL)
		return ERROR;

	if (pdmb_data->data_buf_addr == 0) {
		print_log(NULL, "[1seg] broadcast_drv_if_get_dmb_data[ERR]\
			data_buf is null\n");
		return ERROR;
	}

	if (pdmb_data->data_buf_size < 188) {
		print_log(NULL, "[1seg] broadcast_drv_if_get_dmb_data[ERR]\
			buffsize < 188\n");
		return ERROR;
	}

	total_len = fc8180_get_ts((void*)((unsigned long)pdmb_data->data_buf_addr)
		, pdmb_data->data_buf_size);

	pdmb_data->copied_size = total_len;
	pdmb_data->packet_cnt = TS_BUF_SIZE / 2;

	return OK;
}

int	broadcast_fc8180_drv_if_reset_ch(void)
{
	int rc = OK;

	if (OnAir == 0) {
		print_log(NULL, "[1seg] broadcast_drv_if_reset_ch\
			error [!OnAir]\n");
		return ERROR;
	}

	print_log(NULL, "[1seg]broadcast_drv_if_reset_ch\n");

	return rc;
}

int	broadcast_fc8180_drv_if_user_stop(int mode)
{
	int rc;
	rc = OK;
	if (OnAir == 0) {
		print_log(NULL, "[1seg][no semaphore]\
			broadcast_drv_if_user_stop error [!OnAir]\n");
		return ERROR;
	}

	print_log(NULL, "[1seg][no semaphore] broadcast_drv_if_user_stop\n");
    tunerbb_drv_fc8180_set_user_stop(mode);

	return rc;
}

int	broadcast_fc8180_drv_if_select_antenna(unsigned int sel)
{
	int rc;
	rc = OK;
	if (OnAir == 0) {
		print_log(NULL, "[1seg][no semaphore]\
			broadcast_drv_if_select_antenna error [!OnAir]\n");
		return ERROR;
	}
	return rc;
}

int	broadcast_fc8180_drv_if_isr(void)
{
	int rc;
	rc = OK;
	if (OnAir == 0) {
		print_log(NULL, "[1seg] broadcast_drv_if_isr error [!OnAir]\n");
		return ERROR;
	}
	return rc;
}

int	broadcast_fc8180_drv_if_read_control(char *buf, unsigned int size)
{
	return 0;
}

int	broadcast_fc8180_drv_if_get_mode(unsigned short *mode)
{
	int rc = ERROR;

	if (mode == NULL)
		return ERROR;

	if (OnAir == 0 || currentSelectedChannel == -1)
		mode[0] = 0xFFFF;
	else {
		unsigned short channel_num;
		unsigned short band = 0;
		unsigned short rcvSegment = 0;

		channel_num = currentSelectedChannel;

		if (currentBroadCast == UHF_1SEG) {
			band = 0;
			rcvSegment = 1;
		} else if (currentBroadCast == UHF_13SEG) {
			band = 0;
			rcvSegment = 0;
		} else if (currentBroadCast == TMM_1SEG) {
			band = 1;
			rcvSegment = 1;
		} else {
			band = 1;
			rcvSegment = 0;
		}

		mode[0] = ((channel_num & 0xFF) << 8) |
				    ((band & 0x0F) << 4) |
				    (rcvSegment & 0x0F);
	}

	rc = OK;
	return rc;
}
/*                                                                          */
/*--------------------------------------------------------------------------*/


/* optional part when we include driver code to build-on
it's just used when we make device driver to module(.ko)
so it doesn't work in build-on */
MODULE_DESCRIPTION("FCI ISDB-Tmm device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FCI");
