
/*
 * Driver for EEPROM
 *
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "Common_eeprom.h"

//#define EEPROMGETDLT_DEBUG
#define EEPROM_DEBUG
#ifdef EEPROM_DEBUG
#define EEPROMDB printk
#else
#define EEPROMDB(x,...)
#endif


static DEFINE_SPINLOCK(g_EEPROMLock); // for SMP
#define EEPROM_I2C_BUSNUM 1

/*******************************************************************************
*
********************************************************************************/
#define EEPROM_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define EEPROM_DRVNAME "CAM_CAL_DRV"
#define EEPROM_I2C_GROUP_ID 0

/*******************************************************************************
*
********************************************************************************/
//#define FM50AF_EEPROM_I2C_ID 0x28
#define FM50AF_EEPROM_I2C_ID 0xA1


/*******************************************************************************/
/* define LSC data for M24C08F EEPROM on L10 project */
/********************************************************************************/
#define SampleNum 221
#define Read_NUMofEEPROM 2
#define Boundary_Address 256
#define EEPROM_Address_Offset 0xC


/*******************************************************************************
*
********************************************************************************/
static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_EEPROMdevno = MKDEV(EEPROM_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pEEPROM_CharDrv = NULL;
//static spinlock_t g_EEPROMLock;
//spin_lock(&g_EEPROMLock);
//spin_unlock(&g_EEPROMLock);

static struct class *EEPROM_class = NULL;
static atomic_t g_EEPROMatomic;
//static DEFINE_SPINLOCK(kdeeprom_drv_lock);
//spin_lock(&kdeeprom_drv_lock);
//spin_unlock(&kdeeprom_drv_lock);


#if defined(M24C32_EEPROM)
extern int M24C32_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
extern void read_moduleID_M24C32(unsigned char * pinputdata); // LGE_CHANGE: kyunghun.oh@lge.com, Add Module Vendor ID - bug fix
#endif

#if defined(ZC533_EEPROM)
extern int ZC533_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
extern void read_moduleID_ZC533(unsigned char * pinputdata);
#endif

#if defined(DW9763_EEPROM)
extern int DW9763_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
extern void read_moduleID_DW9763(unsigned char * pinputdata);
#endif

#if defined(ZC535_EEPROM)
extern int ZC535_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
extern void read_moduleID_ZC535(unsigned char * pinputdata);
#endif

#if defined(BRCF016GWZ3_EEPROM)
extern int BRCF016GWZ3_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
extern void read_moduleID_BRCF016GWZ3(unsigned char * pinputdata);
#endif

#if defined(BRCB032GWZ3_EEPROM)
extern int BRCB032GWZ3_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
extern void read_moduleID_BRCB032GWZ3(unsigned char * pinputdata);
#endif

/*******************************************************************************
*
********************************************************************************/

// LGE_CHANGE_S: [2016-02-13] kyunghun.oh@lge.com, Add Module Vendor ID


static struct class *camera_vendor_id_class = NULL;
static s8 main_cam_module_id = -1;
static unsigned char moduleId = 0;

static ssize_t show_LGCameraMainID(struct device *dev,struct device_attribute *attr, char *buf)
{
    EEPROMDB("show_LGCameraMainID: main_camera_id [%d] \n", main_cam_module_id);
    switch (main_cam_module_id)
       {
        case 0x01:
        case 0x02:
        case 0x05:
        case 0x06:
        case 0x07:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "LGIT");
        case 0x03:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "Fujifilm");
        case 0x04:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "Minolta");
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "Cowell");
        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "IM-tech");
        case 0x20:
        case 0x21:
        case 0x22:
        case 0x23:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "Sunny");
        default:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id&0xFF, "Reserved for future");
    }
}
static DEVICE_ATTR(vendor_id, S_IRUGO, show_LGCameraMainID, NULL);
// LGE_CHANGE_E: [2016-02-13] kyunghun.oh@lge.com, Add Module Vendor ID


/*******************************************************************************
*
********************************************************************************/
// maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt6516.c which is 8 bytes


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int EEPROM_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long EEPROM_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf = NULL;

#ifdef EEPROMGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif
    void (*read_moduleID)(unsigned char * pinputdata) = NULL; // LGE_CHANGE: [2016-02-13] kyunghun.oh@lge.com, Add Module Vendor ID

    EEPROMDB("[COMMON_EEPROM]1 In to IOCTL %x %x\n",_IOC_DIR(a_u4Command),_IOC_WRITE);
    EEPROMDB("[COMMON_EEPROM]2 In to IOCTL %x %x\n",CAM_CALIOC_G_READ,CAM_CALIOC_S_WRITE);

	
    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            EEPROMDB("[S24EEPROM] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                EEPROMDB("[S24EEPROM] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
        //LGE_CHANGE_S: [2015-11-18] yonghwan.lym@lge.com, Static_Analysis
        ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
        EEPROMDB("[COMMON_EEPROM] In to IOCTL %x (0x%08x)\n",ptempbuf->u4Offset,(ptempbuf->u4Offset & 0x000FFFFF));
        //LGE_CHANGE_E: [2015-11-18] yonghwan.lym@lge.com, Static_Analysis
    }

    if(ptempbuf == NULL)
    {
        EEPROMDB("[COMMON_EEPROM] ptempbuf is NULL!!\n");
        return -EFAULT;
    }

    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x0000C533)
    {
#if defined(ZC533_EEPROM)
        printk("[ZC533] Jump to IOCTL \n");

        read_moduleID = read_moduleID_ZC533;
        i4RetValue = ZC533_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        printk("[ZC533] Not defined in config \n");
#endif
    }
    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x000CB24C)
    {
#if defined(M24C32_EEPROM)
        printk("[M24C32] Jump to IOCTL \n");

        read_moduleID = read_moduleID_M24C32;
        i4RetValue = M24C32_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        printk("[M24C32] Not defined in config \n");
#endif
    }
    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x00009763)
    {
#if defined(DW9763_EEPROM)
        printk("[DW9763] Jump to IOCTL \n");

        read_moduleID = read_moduleID_DW9763;
        i4RetValue = DW9763_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        printk("[DW9763] Not defined in config \n");
#endif
    }
    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x0000C535)
    {
#if defined(ZC535_EEPROM)
        printk("[ZC535] Jump to IOCTL \n");

        read_moduleID = read_moduleID_ZC535;
        i4RetValue = ZC535_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        printk("[ZC535] Not defined in config \n");
#endif
    }
    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x000cf016)
    {
#if defined(BRCF016GWZ3_EEPROM)
        printk("[BRCF016GWZ3] Jump to IOCTL \n");

        read_moduleID = read_moduleID_BRCF016GWZ3;
        i4RetValue = BRCF016GWZ3_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        printk("[BRCF016GWZ3] Not defined in config \n");
#endif
    }
    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x000CB032)
    {
#if defined(BRCB032GWZ3_EEPROM)
        printk("[BRCB032GWZ3] Jump to IOCTL \n");

        read_moduleID = read_moduleID_BRCB032GWZ3;
        i4RetValue = BRCB032GWZ3_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        printk("[BRCB032GWZ3] Not defined in config \n");
#endif
    }
    else
        {
            EEPROMDB("[COMMON_EEPROM] Masic number is wrong \n");
        }

    kfree(pBuff);

	// LGE_CHANGE_S: [2015-11-23] yonghwan.lym@lge.com, Add Module Vendor ID - bug fix
	if((read_moduleID != NULL) && (!moduleId))
	{
		read_moduleID(&moduleId);
        main_cam_module_id =  moduleId;
	}
	// LGE_CHANGE_E: [2016-02-13] kyunghun.oh@lge.com, Add Module Vendor ID - bug fix

    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int EEPROM_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    printk("[COMMON_EEPROM] EEPROM_Open Client %p\n",g_pstI2Cclient);
    spin_lock(&g_EEPROMLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_EEPROMLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_EEPROMatomic,0);
    }
    spin_unlock(&g_EEPROMLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int EEPROM_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_EEPROMLock);

    g_u4Opened = 0;

    atomic_set(&g_EEPROMatomic,0);

    spin_unlock(&g_EEPROMLock);

    return 0;
}

static const struct file_operations g_stEEPROM_fops =
{
    .owner = THIS_MODULE,
    .open = EEPROM_Open,
    .release = EEPROM_Release,
    //.ioctl = EEPROM_Ioctl
    .unlocked_ioctl = EEPROM_Ioctl
};

#define EEPROM_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterEEPROMCharDrv(void)
{
    struct device* EEPROM_device = NULL;
	struct device* camera_vendor_id_dev =NULL; // LGE_CHANGE: [2015-11-09] yonghwan.lym@lge.com, Add Module Vendor ID
#if EEPROM_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_EEPROMdevno, 0, 1,EEPROM_DRVNAME) )
    {
        EEPROMDB("[S24EEPROM] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_EEPROMdevno , 1 , EEPROM_DRVNAME) )
    {
        EEPROMDB("[COMMON_EEPROM] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pEEPROM_CharDrv = cdev_alloc();

    if(NULL == g_pEEPROM_CharDrv)
    {
        unregister_chrdev_region(g_EEPROMdevno, 1);

        EEPROMDB("[COMMON_EEPROM] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pEEPROM_CharDrv, &g_stEEPROM_fops);

    g_pEEPROM_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pEEPROM_CharDrv, g_EEPROMdevno, 1))
    {
        EEPROMDB("[COMMON_EEPROM] Attatch file operation failed\n");

        unregister_chrdev_region(g_EEPROMdevno, 1);

        return -EAGAIN;
    }

    EEPROM_class = class_create(THIS_MODULE, "EEPROMdrv");
    if (IS_ERR(EEPROM_class)) {
        int ret = PTR_ERR(EEPROM_class);
        EEPROMDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    EEPROM_device = device_create(EEPROM_class, NULL, g_EEPROMdevno, NULL, EEPROM_DRVNAME);
	    camera_vendor_id_class = class_create(THIS_MODULE, "camera");
	    camera_vendor_id_dev = device_create(camera_vendor_id_class, NULL, 0, NULL, "vendor_id");
	    device_create_file(camera_vendor_id_dev, &dev_attr_vendor_id);
    return 0;
}

inline static void UnregisterEEPROMCharDrv(void)
{
    //Release char driver
    cdev_del(g_pEEPROM_CharDrv);

    unregister_chrdev_region(g_EEPROMdevno, 1);

    device_destroy(EEPROM_class, g_EEPROMdevno);
    class_destroy(EEPROM_class);
}


static int EEPROM_probe(struct platform_device *pdev)
{
    return 0;
}

static int EEPROM_remove(struct platform_device *pdev)
{
    return 0;
}


// platform structure
static struct platform_driver g_stEEPROM_Driver = {
    .probe		= EEPROM_probe,
    .remove	= EEPROM_remove,
    .driver		= {
        .name	= EEPROM_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stEEPROM_Device = {
    .name = EEPROM_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init EEPROM_i2C_init(void)
{
    RegisterEEPROMCharDrv();

    if (platform_device_register(&g_stEEPROM_Device))
    {
        EEPROMDB("failed to register S24EEPROM driver, 2nd time\n");
        return -ENODEV;
    }


    if(platform_driver_register(&g_stEEPROM_Driver)){
        EEPROMDB("failed to register S24EEPROM driver\n");
        return -ENODEV;
    }

	

    return 0;
}

static void __exit EEPROM_i2C_exit(void)
{
	platform_driver_unregister(&g_stEEPROM_Driver);
}

module_init(EEPROM_i2C_init);
module_exit(EEPROM_i2C_exit);

MODULE_DESCRIPTION("EEPROM driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


