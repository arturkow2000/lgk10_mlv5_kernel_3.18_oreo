#ifndef __MACH_EFUSE_MT6735_53_H__
#define __MACH_EFUSE_MT6735_53_H__

// -------------------------------
// Structure
// -------------------------------
typedef enum {
	EFUSE_IDX_COMMON_CTRL       = 0,
	EFUSE_IDX_SECURE_CTRL       = 1,
//	EFUSE_IDX_C_CTRL_1          = 2,	// Not use
	EFUSE_IDX_C_CTRL_0          = 3,
	EFUSE_IDX_C_LOCK            = 4,
	EFUSE_IDX_SECURE_LOCK       = 5,
	EFUSE_IDX_COMMON_LOCK       = 6,
	EFUSE_IDX_MAX               = 6
} efuse_idx;

typedef enum {
	EFUSE_RESULT_C_CTRL_0         = 0,  // Enable modem sbc
	EFUSE_RESULT_SECURE_CTRL      = 1,  // Blow secure related fuses
	EFUSE_RESULT_COMMON_LOCK      = 2,  // Disable common control blow
	EFUSE_RESULT_SECURE_LOCK      = 3,  // Disable secure control blow
	EFUSE_RESULT_SBC_PUBK_HASH    = 4,  // Use oem public key hash
	EFUSE_RESULT_CUSTOM_PUB_KEY   = 5,  // Prevent cross-download
	EFUSE_RESULT_COMMON_CTRL      = 6,  // Blow common fuses such as boot, USB
	EFUSE_RESULT_C_LOCK           = 7,  // Disable C ctrl & data blow
	EFUSE_RESULT_MAX
} efuse_result_etype;

typedef struct {
	u32 idx;
	u32 r_type;    // result_type
	u32 k_index;   // kernel index
	u32 b_mask;    // mask bits
} struct_efuse_list;

typedef struct {
	u32 seq;
	u32 k_index;             // kernel index
	u32 b_mask;              // mask bits
	u32 mtk_key;             // mtk default value
	u32 lge_default_key;     // lge default value
} struct_pubk_list;

typedef struct {
    u32 magic[2];
	u32 control;
	u32 ac_key[4];
	u32 sbc_pubk_hash[8];
	u32 usb_id[2];
	u32 c_data[4];
	u32 config[7];
	u32 padding[92];
	u32 partition_hash[8];
} struct_efuse_all_partition_all;

typedef struct {
	u32 pubk_hash[8];
	u32 config[7];
} struct_efuse_using_partition;

// -------------------------------
// define
// -------------------------------
#define SHA256_HASH_SIZE 32

#define SBC_KERNEL_INDEX   6
#define SBC_KERNEL_MASKING 0x2

#define MODULE_NAME "lge-qfprom"
#define MODULE_TABLE_NAME "lge,lge-qfprom"

#define EFUSE_PARTITION_NAME "efuse"
#define EFUSE_PARTITION_READ_SIZE 8    // 32 bits
#define EFUSE_PARTITION_USED_SIZE 512  // Efuse partition size : 512KB. But, real used size is 512 bytes.

#define EFUSE_PARTITION_OFFSET_SBC_PUBK_HASH 0x1C
#define EFUSE_PARTITION_OFFSET_CONFIG        0x54
#define EFUSE_MAGIC_CODE1                    0x32715131
#define EFUSE_MAGIC_CODE2                    0x501409A7

#define RET_OK 0
#define RET_ERR 1

#define IDX_ARB_GROUP_0 24
#define IDX_ARB_GROUP_1 25

// -------------------------------
// Variables
// -------------------------------
static struct_efuse_list fuse_list[EFUSE_IDX_MAX] = {
	{EFUSE_IDX_COMMON_CTRL,      EFUSE_RESULT_COMMON_CTRL,       0, 0x00000017},
	{EFUSE_IDX_SECURE_CTRL,      EFUSE_RESULT_SECURE_CTRL,       6, 0x000002FF},
	{EFUSE_IDX_C_CTRL_0,         EFUSE_RESULT_C_CTRL_0,         33, 0x00000007},
	{EFUSE_IDX_C_LOCK,           EFUSE_RESULT_C_LOCK,           62, 0x00000033},
	{EFUSE_IDX_SECURE_LOCK,      EFUSE_RESULT_SECURE_LOCK,      63, 0x00000007},
	{EFUSE_IDX_COMMON_LOCK,      EFUSE_RESULT_COMMON_LOCK,      64, 0x00000006}
};

static struct_pubk_list pubk_list[8] = {
	{0, 50, 0xFFFFFFFF, 0xE7E272C4, 0x0F2BDBEF},
	{1, 51, 0xFFFFFFFF, 0xD42B217D, 0xC4E52A01},
	{2, 52, 0xFFFFFFFF, 0x71E33163, 0x2CFF2C27},
	{3, 53, 0xFFFFFFFF, 0x84BEE069, 0x1AAAA98B},
	{4, 54, 0xFFFFFFFF, 0x33BDA647, 0x5F662ACE},
	{5, 55, 0xFFFFFFFF, 0x80C50624, 0xC7BE7765},
	{6, 56, 0xFFFFFFFF, 0x6FF05606, 0x1B77F79C},
	{7, 57, 0xFFFFFFFF, 0x7A098E8E, 0xBDA8A101}
};

static bool b_efuse_read = false;
static struct mutex secdat_lock;

static struct_efuse_using_partition efuse_data;
static unsigned char hash_p[SHA256_HASH_SIZE] = {0x0, };

// -------------------------------
// functions
// -------------------------------

#endif // __MACH_EFUSE_MT6735_53_H__
