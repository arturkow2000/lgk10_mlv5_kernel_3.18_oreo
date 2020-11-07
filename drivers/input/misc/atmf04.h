#ifndef __ATMF04_EFLASH_H__
#define __ATMF04_EFLASH_H__

#define SZ_PAGE_DATA                64
#ifdef CONFIG_LGE_USE_CAP_SENSOR
#define FW_DATA_PAGE               	115
#else
#define FW_DATA_PAGE               	96
#endif

#define ADDR_EFLA_STS               0xFF	//eflash status register
#define ADDR_EFLA_PAGE_L            0xFD	//eflash page
#define ADDR_EFLA_PAGE_H            0xFE	//eflash page
#define ADDR_EFLA_CTRL              0xFC	//eflash control register

#define CMD_EFL_L_WR                0x01	//Eflash Write
#define CMD_EFL_RD                  0x03	//Eflash Read
#define CMD_EFL_ERASE_ALL           0x07	//Eflash All Page Erase

#define CMD_EUM_WR                  0x21	//Extra user memory write
#define CMD_EUM_RD                  0x23	//Extra user memory read
#define CMD_EUM_ERASE               0x25	//Extra user memory erase

#define FLAG_DONE                   0x03
#define FLAG_DONE_ERASE             0x02

#define FLAG_FUSE                   1
#define FLAG_FW                     2

#define FL_EFLA_TIMEOUT_CNT         20

#define RTN_FAIL                    0
#define RTN_SUCC                    1
#define RTN_TIMEOUT                 2

#define ON                          1
#define OFF                         2

#if 1 // debugging calibration paused
#define CAP_CAL_RESULT_PASS			0 // "1"
#define CAP_CAL_RESULT_FAIL			"0"
#endif

#endif

/* If you use 2 channel on a cap sensor, use below define. */
#define CONFIG_LGE_ATMF04_2CH

#if defined(CONFIG_LGE_USE_CAP_SENSOR) // register init value
#if defined(CONFIG_LGE_ATMF04_2CH)
#define CNT_INITCODE               26
#else
#define CNT_INITCODE               17
#endif
// Each operator use different initcode value
#if defined(CONFIG_LGE_ATMF04_2CH)
static const unsigned char InitCodeAddr[CNT_INITCODE]    = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x09, 0x0A, 0x0B, 0X0C, 0X0D, 0x0E, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D };
static const unsigned char InitCodeVal[CNT_INITCODE]     = { 0x00, 0x83, 0x00, 0x25, 0x33, 0x0B, 0x0B, 0x91, 0x73, 0x91, 0x7D, 0x5B, 0x83, 0x00, 0x42, 0x00, 0x10, 0xD0, 0xA4, 0x31, 0x20, 0x0B, 0x05, 0x34, 0x12, 0x23 };
#else // default
static const unsigned char InitCodeAddr[CNT_INITCODE]    = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x0C, 0x0D, 0x0E, 0x1A, 0x1B, 0x1C, 0x1D, 0x20, 0x21 };
static const unsigned char InitCodeVal[CNT_INITCODE]     = { 0x00, 0x7A, 0x33, 0x0B, 0x08, 0x6B, 0x68, 0x17, 0x00, 0x14, 0x7F, 0x00, 0x0B, 0x00, 0x07, 0x81, 0x20 }; // High Band ANT , auto cal 15%, sensing 2.5%, LNF filter 
#endif
#else // defined (CONFIG_LGE_USE_CAP_SENSOR)
#define CNT_INITCODE                7
const unsigned char InitCodeAddr[CNT_INITCODE] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
const unsigned char InitCodeVal[CNT_INITCODE] = { 0x00, 0x0A, 0x69, 0x67, 0x0B, 0x33, 0x1E };
#endif // defined (CONFIG_LGE_USE_CAP_SENSOR)