/* 
 * ICM406XX sensor driver
 * Copyright (C) 2018 Invensense, Inc.
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
 
#if defined(CONFIG_OF)
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#endif
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <asm/siginfo.h>
#include <linux/miscdevice.h>
#include "../step_counter.h"
#include "../../accelerometer/icm406xx_a/icm406xx_register.h"
#include "../../accelerometer/icm406xx_a/icm406xx_share.h"
#include "icm406xx_sc.h"

static atomic_t icm406xx_sc_trace;
static int icm406xx_sc_init_flag =  -1;

static struct platform_device *pltfm_dev;
struct platform_device *get_icm406xx_platformdev(void)
{
    return pltfm_dev;
}

static int icm406xx_sc_local_init(void);
static int icm406xx_sc_local_remove(void);
static struct step_c_init_info icm406xx_sc_init_info = {
    .name = "ICM406XX_SC",
    .init = icm406xx_sc_local_init,
    .uninit = icm406xx_sc_local_remove,
};

struct icm406xx_sc_context {
    int irq;
    struct hrtimer hrTimer;
    int is_timer_running;
    struct work_struct  pollwork;
    struct task_struct *daemon_t;
    int wom_count;
    int wom_thres;
    int wom_int_detected;
    int sc_enabled;
    int sd_enabled;
    int smd_enabled;
    int sc_value;
    s16 accel_data[ICM406XX_AXIS_NUM];
};

static struct icm406xx_sc_context *g_sc_obj_data;

static int icm406xx_sc_EnableStepCounter(u8 en)
{
    struct icm406xx_sc_context *obj = NULL;
    u8 databuf[2] = {0};
    int res = 0;

    obj = g_sc_obj_data;
    obj->sc_enabled = en;
    obj->wom_count = ICM406XX_WOM_COUNT;
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_SC, en);
    if (res != ICM406XX_SUCCESS)
        return res;
    res = icm406xx_share_read_register(REG_SMD_CONFIG, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("read smd config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    if (en) {
        res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_SC,
            DEFAULT_SAMPLING_PERIOD_NS, false);
        if (res != ICM406XX_SUCCESS)
            return res;
        databuf[0] |= (BIT_WOM_INT_MODE_OR |
            BIT_WOM_MODE_PREV |
            BIT_SMD_MODE_OLD);
        if (!obj->sd_enabled) {
            hrtimer_cancel(&obj ->hrTimer);
            hrtimer_start(&obj ->hrTimer, ns_to_ktime(0), HRTIMER_MODE_REL);
            obj->is_timer_running = 1;
            res = icm406xx_share_EnableInterrupt(ICM406XX_INT_TYPE_WOM, en);
            if (res != ICM406XX_SUCCESS)
                return res;
        }
        if (obj->irq)
            enable_irq_wake(obj->irq);
    } else {
        res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_SC, 0, false);
        if (res != ICM406XX_SUCCESS)
            return res;
        if (!obj->sd_enabled) {
            databuf[0] &= ~(BIT_WOM_INT_MODE_OR |
                BIT_WOM_MODE_PREV |
                BIT_SMD_MODE_OLD);
            hrtimer_cancel(&obj ->hrTimer);
            obj->is_timer_running = 0;
            res = icm406xx_share_EnableInterrupt(ICM406XX_INT_TYPE_WOM, en);
            if (res != ICM406XX_SUCCESS)
                return res;
        }
        if (!icm406xx_share_any_accel_based_sensor_is_on() && obj->irq)
            disable_irq_wake(obj->irq);
    }
    if (obj->smd_enabled)
        databuf[0] |= (BIT_SMD_MODE_LONG);
    res = icm406xx_share_write_register(REG_SMD_CONFIG, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("write smd config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    return res;
}

static int icm406xx_sc_EnableStepDetector(u8 en)
{
    struct icm406xx_sc_context *obj = NULL;
    u8 databuf[2] = {0};
    int res = 0;

    obj = g_sc_obj_data;
    obj->sd_enabled = en;
    obj->wom_count = ICM406XX_WOM_COUNT;
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_SD, en);
    if (res != ICM406XX_SUCCESS)
        return res;
    res = icm406xx_share_read_register(REG_SMD_CONFIG, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("read smd config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    if (en) {
        res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_SD,
            DEFAULT_SAMPLING_PERIOD_NS, false);
        if (res != ICM406XX_SUCCESS)
            return res;
        databuf[0] |= (BIT_WOM_INT_MODE_OR |
            BIT_WOM_MODE_PREV |
            BIT_SMD_MODE_OLD);
        if (!obj->sc_enabled) {
            hrtimer_cancel(&obj ->hrTimer);
            hrtimer_start(&obj ->hrTimer, ns_to_ktime(0), HRTIMER_MODE_REL);
            obj->is_timer_running = 1;
            res = icm406xx_share_EnableInterrupt(ICM406XX_INT_TYPE_WOM, en);
            if (res != ICM406XX_SUCCESS)
                return res;
        }
        if (obj->irq)
            enable_irq_wake(obj->irq);
    } else {
        res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_SD, 0, false);
        if (res != ICM406XX_SUCCESS)
            return res;
        if (!obj->sc_enabled) {
            databuf[0] &= ~(BIT_WOM_INT_MODE_OR |
                BIT_WOM_MODE_PREV |
                BIT_SMD_MODE_OLD);
            hrtimer_cancel(&obj ->hrTimer);
            obj->is_timer_running = 0;
            res = icm406xx_share_EnableInterrupt(ICM406XX_INT_TYPE_WOM, en);
            if (res != ICM406XX_SUCCESS)
                return res;
        }
        if (!icm406xx_share_any_accel_based_sensor_is_on() && obj->irq)
            disable_irq_wake(obj->irq);
    }
    if (obj->smd_enabled)
        databuf[0] |= (BIT_SMD_MODE_LONG);
    res = icm406xx_share_write_register(REG_SMD_CONFIG, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("write smd config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    return res;
}

static int icm406xx_sc_EnableSMD(u8 en)
{
    struct icm406xx_sc_context *obj = NULL;
    u8 databuf[2] = {0};
    int res = 0;

    obj = g_sc_obj_data;
    obj->smd_enabled = en;
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_SMD, en);
    if (res != ICM406XX_SUCCESS)
        return res;
    res = icm406xx_share_EnableInterrupt(ICM406XX_INT_TYPE_SMD, en);
    if (res != ICM406XX_SUCCESS)
        return res;
    res = icm406xx_share_read_register(REG_SMD_CONFIG, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("read smd config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    if (en) {
        databuf[0] |= (BIT_SMD_MODE_LONG);
        res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_SMD,
            DEFAULT_SAMPLING_PERIOD_NS, false);
        if (res != ICM406XX_SUCCESS)
            return res;
        if (obj->irq)
            enable_irq_wake(obj->irq);
    } else {
        databuf[0] &= ~(BIT_SMD_MODE_LONG);
        res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_SMD, 0, false);
        if (res != ICM406XX_SUCCESS)
            return res;
        if (!icm406xx_share_any_accel_based_sensor_is_on() && obj->irq)
            disable_irq_wake(obj->irq);
    }
    if (obj->sc_enabled || obj->sd_enabled)
        databuf[0] |= (BIT_SMD_MODE_OLD);
    res = icm406xx_share_write_register(REG_SMD_CONFIG, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("write smd config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    return res;
}

static int icm406xx_sc_InterruptConfig(void)
{
    u8 databuf[2] = {0};
    int res = 0;

    databuf[0] = (INT_POLARITY << SHIFT_INT1_POLARITY) |
        (INT_DRIVE_CIRCUIT << SHIFT_INT1_DRIVE_CIRCUIT) |
        (INT_MODE << SHIFT_INT1_MODE);
    res = icm406xx_share_write_register(REG_INT_CONFIG_REG, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("write interrupt config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    return ICM406XX_SUCCESS;
}

static int icm406xx_sc_ThresholdWOM(int mg)
{
    u8 databuf[2] = {0};
    int res = 0;

    databuf[0] = ICM406XX_WOM_COMPUTE(mg);
    res = icm406xx_share_write_register(REG_ACCEL_WOM_X_THR, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("write wom config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    res = icm406xx_share_write_register(REG_ACCEL_WOM_Y_THR, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("write wom config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    res = icm406xx_share_write_register(REG_ACCEL_WOM_Z_THR, databuf, 1);
    if (res < 0) {
        STEP_C_ERR("write wom config failed!\n");
        return ICM406XX_ERR_BUS;
    }
    return ICM406XX_SUCCESS;
}

static int icm406xx_sc_ReadSensorData(uint32_t *value)
{
    struct icm406xx_sc_context *obj = NULL;

    obj = g_sc_obj_data;
    *value = obj->sc_value;
    return 0;
}

void icm406xx_sc_NotifySensorData(int int_type)
{
    if (int_type == ICM406XX_SENSOR_TYPE_SD) {
        step_notify(TYPE_STEP_DETECTOR);
        STEP_C_LOG("Step Detector Nofity - %d\n", int_type);
    }
    if (int_type == ICM406XX_SENSOR_TYPE_SMD) {
        step_notify(TYPE_SIGNIFICANT);
        STEP_C_LOG("Significant Motion Nofity - %d\n", int_type);
    }
}

static int icm406xx_sc_init_client(void)
{
    int res = 0;
    /* exit sleep mode */
    res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SC, true);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set interrupt config */
    res = icm406xx_sc_InterruptConfig();
    if (res != ICM406XX_SUCCESS)
        return res;    
    /* set wom thershold value */
    res = icm406xx_sc_ThresholdWOM(ICM406XX_WOM_DEFAULT_THRESHOLD);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set 40ms(25hz) sample rate */
    res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_SC,
        DEFAULT_SAMPLING_PERIOD_NS, false);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* disable sensor */
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_SC, false);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* disable sensor */
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_SD, false);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* disable sensor */
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_SMD, false);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set power mode - sleep or normal */
    res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SC, false);
    if (res != ICM406XX_SUCCESS)
        return res;
    STEP_C_LOG("icm406xx_sc_init_client OK!\n");
    return ICM406XX_SUCCESS;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res = 0;

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n",
        atomic_read(&icm406xx_sc_trace));
    return res;
}

static ssize_t store_trace_value(struct device_driver *ddri,
    const char *buf, size_t count)
{
    int trace;

    if (!kstrtoint(buf, 16, &trace))
        atomic_set(&icm406xx_sc_trace, trace);
    else
        STEP_C_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t res = 0;
    return res;
}

#ifdef ICM406XX_WOM_THRESHOLD_TUNE
static ssize_t show_wom_threshold(struct device_driver *ddri, char *buf)
{
    struct icm406xx_sc_context *obj = NULL;
    ssize_t len = 0;

    obj = g_sc_obj_data;
    len = snprintf(buf, PAGE_SIZE, "0x%02X\n", obj->wom_thres);
    return len;
}

static ssize_t store_wom_threshold(struct device_driver *ddri,
    const char *buf, size_t count)
{
    int value;	
    int res = 0;

    icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SC, true);
    res = kstrtoint(buf, 10, &value);
    if (res)
        return count;
    /* set wom thershold value */
    res = icm406xx_sc_ThresholdWOM(value);
    if (res != ICM406XX_SUCCESS)
        return res;
    return count;
}
#endif

static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value,
    store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
#ifdef ICM406XX_WOM_THRESHOLD_TUNE
static DRIVER_ATTR(womthres, S_IWUSR | S_IRUGO,
    show_wom_threshold, store_wom_threshold);
#endif
static struct driver_attribute *icm406xx_sc_attr_list[] = {
    /*trace log*/
    &driver_attr_trace,
    &driver_attr_status,
#ifdef ICM406XX_WOM_THRESHOLD_TUNE
    &driver_attr_womthres,
#endif
};

static int icm406xx_sc_create_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = (int)(sizeof(icm406xx_sc_attr_list)/
        sizeof(icm406xx_sc_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++) {
        res = driver_create_file(driver, icm406xx_sc_attr_list[idx]);
        if (0 != res) {
            STEP_C_ERR("driver_create_file (%s) = %d\n",
                icm406xx_sc_attr_list[idx]->attr.name, res);
            break;
        }
    }
    return res;
}

static int icm406xx_sc_delete_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = (int)(sizeof(icm406xx_sc_attr_list)/
        sizeof(icm406xx_sc_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, icm406xx_sc_attr_list[idx]);
    return res;
}

static int icm406xx_sc_open(struct inode *inode, struct file *file)
{
    file->private_data = g_sc_obj_data;
    if (file->private_data == NULL) {
        STEP_C_ERR("null pointer!!\n");
        return -EINVAL;
    }	
    return nonseekable_open(inode, file);
}

static int icm406xx_sc_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static long icm406xx_sc_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    struct icm406xx_sc_context *obj = 
        (struct icm406xx_sc_context *)file->private_data;
    char string[128];
    void __user *data = (void __user*)arg;
    int pid;
    long res = 0;

    if (_IOC_DIR(cmd) & _IOC_READ)
        res = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        res = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (res) {
        STEP_C_ERR("access error: %08X, (%2d, %2d)\n", 
            cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }
    switch (cmd) {
    case ICM406XX_WRITE_DAEMON_PID:
        res = copy_from_user(&pid, data, sizeof(int));
        if (res != 0)
            return -EINVAL;
        obj->daemon_t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);       
        if (obj->daemon_t == NULL)
            return -ENODEV;
        break;
    case ICM406XX_READ_SENSOR_DATA:
        res = copy_to_user(data, obj->accel_data, sizeof(obj->accel_data));
        if (res != 0)
            return -EINVAL;
        break;
    case ICM406XX_WRITE_SENSOR_DATA:
        res = copy_from_user(&obj->sc_value, data, sizeof(obj->sc_value));
        if (res != 0)
            return -EINVAL;
        if (obj->sd_enabled)
            icm406xx_sc_NotifySensorData(ICM406XX_SENSOR_TYPE_SD);
        break;
    case ICM406XX_LOG:
        res = copy_from_user(&string, data, 128);           
        printk("%s",string);
        break;
    default:
        STEP_C_ERR("unknown IOCTL: 0x%08x\n", cmd);
        res = -ENOIOCTLCMD;
    }
    return res;
}

static const struct file_operations icm406xx_sc_fops = {
    .open = icm406xx_sc_open,
    .release = icm406xx_sc_release,
    .unlocked_ioctl = icm406xx_sc_unlocked_ioctl,
};

static struct miscdevice icm406xx_sc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "icm406xx_sc",
    .fops = &icm406xx_sc_fops,
};

static int icm406xx_sc_open_report_data(int open)
{
    return 0;
}

static int icm406xx_sc_enable_nodata(int en)
{
    if (1 == en) {
        icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SC, true);
        icm406xx_sc_EnableStepCounter(true);
    } else {
        icm406xx_sc_EnableStepCounter(false);
        icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SC, false);
    }
    STEP_C_LOG("icm406xx_sc_enable_nodata %d\n", en);
    return 0;
}

static int icm406xx_sc_enable_step_detect(int en)
{
    if(1 == en) {
        icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SD, true);
        icm406xx_sc_EnableStepDetector(true);
    } else {
        icm406xx_sc_EnableStepDetector(false);
        icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SD, false);
    }
    STEP_C_LOG("icm406xx_sc_enable_step_detect %d\n", en);
    return 0;
}

static int icm406xx_sc_enable_significant(int en)
{
    if(1 == en) {
        icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SMD, true);
        icm406xx_sc_EnableSMD(true);
    } else {
        icm406xx_sc_EnableSMD(false);
        icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SMD, false);
    }
    STEP_C_LOG("icm406xx_sc_enable_significant %d\n", en);
    return 0;
}

static enum hrtimer_restart icm406xx_sc_poll(struct hrtimer *timer)
{
    struct icm406xx_sc_context *obj = 
        container_of(timer, struct icm406xx_sc_context, hrTimer);

    if ((obj->wom_count > 0 && (obj->sc_enabled || obj->sd_enabled)) ||
        obj->wom_int_detected) {
        hrtimer_forward_now(&obj ->hrTimer,
            ns_to_ktime(DEFAULT_SAMPLING_PERIOD_NS));
        schedule_work(&obj->pollwork);
        return HRTIMER_RESTART;
    }
    obj->is_timer_running = 0;
    return HRTIMER_NORESTART;
}

int icm406xx_sc_signal_to_daemon(int signal)
{
    struct icm406xx_sc_context *obj = g_sc_obj_data;
    struct siginfo info;
    int res = 0;
    
    if (!obj->daemon_t) {
        res = -EINVAL;
        return res;
    }
    memset(&info, 0, sizeof(struct siginfo));
    info.si_signo = SIG_ICM406XX;
    info.si_code = SI_QUEUE;
    info.si_int = signal;
    res = send_sig_info(SIG_ICM406XX, &info, obj->daemon_t);
    if (res < 0) {
        return res;
    }
    return res;
}

static void icm406xx_sc_work_func(struct work_struct *work)
{
    struct icm406xx_sc_context *obj =
        container_of(work, struct icm406xx_sc_context, pollwork);
    char databuf[6];
    u8 int_status2;
    int i;
    int res = 0;

    if (obj->wom_int_detected) {
        obj->wom_int_detected = false;
        res = icm406xx_share_read_register(REG_INT_STATUS2, databuf, 1);
        if (res < 0) {
            STEP_C_ERR("read status2 register err!\n");
            return;
        }
        int_status2 = databuf[0];
        if (int_status2 & BIT_INT_SMD_INT1_EN) {
            /* SMD is oneshot, notify event and disable */
            icm406xx_sc_NotifySensorData(ICM406XX_SENSOR_TYPE_SMD);
            icm406xx_sc_EnableSMD(false);
            icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_SMD, false);
        }
        if (int_status2 & BIT_INT_WOM_XYZ_INT1_EN)
            obj->wom_count = ICM406XX_WOM_COUNT;
    }
    res = icm406xx_share_read_register(REG_ACCEL_DATA_X0_UI, 
        databuf, ICM406XX_DATA_LEN);
    if (res < 0) {
        STEP_C_ERR("read step accel data error\n");
        return;
    }
    /*  convert 8-bit to 16-bit and push data into cache*/
    for (i = 0; i < ICM406XX_AXIS_NUM; i++)
        obj->accel_data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
    /* request daemon to process accel data */
    icm406xx_sc_signal_to_daemon(REQUEST_SIGNAL_PROCESS_DATA);
    if (obj->sc_enabled || obj->sd_enabled)
        obj->wom_count --;
    else
        obj->wom_count = 0;
}

irqreturn_t icm406xx_sc_eint_work(int irq, void *dev_id)
{
    struct icm406xx_sc_context *obj = (struct icm406xx_sc_context *)dev_id;

    obj->wom_int_detected = true;
    if (obj->is_timer_running == 0) {
        obj->is_timer_running = 1;
        hrtimer_cancel(&obj ->hrTimer);
        hrtimer_start(&obj ->hrTimer, ns_to_ktime(0), HRTIMER_MODE_REL);
    }
    return IRQ_HANDLED;
}

static irqreturn_t icm406xx_sc_eint_handler(int irq, void *arg)
{
    /* call irq thread */
    return IRQ_WAKE_THREAD;
}

static int icm406xx_sc_setup_eint(void)
{
    struct icm406xx_sc_context *obj = g_sc_obj_data;
    struct device_node *node;
    u32 ints[2] = {0, 0};
//    struct platform_device *icm406xxPltFmDev;
//    struct pinctrl *pinctrl;
//    struct pinctrl_state *pins_default;
//    struct pinctrl_state *pins_cfg;
    int res = 0;

 /*   icm406xxPltFmDev = get_icm406xx_platformdev();
    pinctrl = devm_pinctrl_get(&icm406xxPltFmDev->dev);
    if (IS_ERR(pinctrl)) {
        res = PTR_ERR(pinctrl);
        STEP_C_ERR("Cannot find icm406xx pinctrl!\n");
    }
    pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
    if (IS_ERR(pins_default)) {
        res = PTR_ERR(pins_default);
        STEP_C_ERR("Cannot find icm406xx pinctrl default!\n");
    }
    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
        res = PTR_ERR(pins_cfg);
        STEP_C_ERR("Cannot find icm406xx pinctrl pin_cfg!\n");
    }
    pinctrl_select_state(pinctrl, pins_cfg);
*/
    node = of_find_compatible_node(NULL, NULL, "mediatek, gse_1-eint");
    if (node) {
        of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);
        obj->irq = irq_of_parse_and_map(node, 0);
        if (!obj->irq) {
            STEP_C_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        res = request_threaded_irq(obj->irq,
            icm406xx_sc_eint_handler,
            icm406xx_sc_eint_work,
            IRQF_TRIGGER_RISING | IRQF_SHARED, "gse_1-eint",
            obj);
        if(res) {
            STEP_C_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
        enable_irq(obj->irq);
    }
    return 0;
}

static int icm406xx_sc_step_c_set_delay(u64 delay)
{
    /* nothing to do here */
    return 0;
}

static int icm406xx_sc_step_d_set_delay(u64 delay)
{
    /* nothing to do here */
    return 0;
}

static int icm406xx_sc_get_data_step_c(uint32_t *value, int *status)
{
    uint32_t step_count = 0;
    icm406xx_sc_ReadSensorData(&step_count);
    *value = step_count;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

static int icm406xx_sc_get_data_step_d(uint32_t *value, int *status)
{
    /* nothing to do here */
    return 0;
}

static int icm406xx_sc_get_data_significant(uint32_t *value, int *status)
{
    /* nothing to do here */
    return 0;
}

static int icm406xx_sc_attr_create(void)
{
    struct icm406xx_sc_context *obj;
    struct step_c_control_path ctl = {0};
    struct step_c_data_path data = {0};
    int res = 0;

    STEP_C_LOG();
    obj = kzalloc(sizeof(*obj), GFP_KERNEL);
    if (!obj) {
        res = -ENOMEM;
        goto exit;
    }
    g_sc_obj_data = obj;
    /* init values */
    obj->irq = 0;
    obj->is_timer_running = 0;
    obj->wom_count = 0;
    obj->wom_thres = ICM406XX_WOM_DEFAULT_THRESHOLD;
    obj->wom_int_detected = false;
    obj->sc_enabled = false;
    obj->sd_enabled = false;
    obj->smd_enabled = false;
    obj->sc_value = 0;
    /* configure hrtimer */
    hrtimer_init(&obj->hrTimer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
    obj->hrTimer.function = icm406xx_sc_poll;
    INIT_WORK(&obj->pollwork, icm406xx_sc_work_func);
    /* setup regs */
    res = icm406xx_sc_init_client();
    if (res)
        goto exit_init_failed;
    /* daemon */
    obj->daemon_t = NULL;
    res = misc_register(&icm406xx_sc_device);
    if (res) {
        STEP_C_ERR("icm406xx_sc_device misc register failed!\n");
        goto exit_misc_device_register_failed;
    }
    /* configure interrupt */
    res = icm406xx_sc_setup_eint();    
    if (res) {
        STEP_C_ERR("setup eint fail = %d\n", res);
        goto exit_init_failed;
    }
    /* create platform_driver attribute */
    res = icm406xx_sc_create_attr(
        &(icm406xx_sc_init_info.platform_diver_addr->driver));
    if (res) {
        STEP_C_ERR("icm406xx_sc create attribute err = %d\n", res);
        goto exit_create_attr_failed;
    }
    /* Fill the step_c_control_path */
    ctl.is_report_input_direct = false;
//    ctl.is_support_batch = false;
    ctl.open_report_data = icm406xx_sc_open_report_data;
    ctl.enable_nodata = icm406xx_sc_enable_nodata;
    ctl.step_c_set_delay  = icm406xx_sc_step_c_set_delay;
    ctl.step_d_set_delay  = icm406xx_sc_step_d_set_delay;
    ctl.enable_significant  = icm406xx_sc_enable_significant;
    ctl.enable_step_detect  = icm406xx_sc_enable_step_detect;
    /* register the step_c_control_path */
    res = step_c_register_control_path(&ctl);
    if (res) {
        STEP_C_ERR("register step_c control path err\n");
        goto exit_kfree;
    }
    /* Fill the step_c_data_path */
    data.get_data = icm406xx_sc_get_data_step_c;
    data.get_data_step_d = icm406xx_sc_get_data_step_d;
    data.get_data_significant = icm406xx_sc_get_data_significant;
    data.vender_div = 1;
    /* register the step_c_data_path */
    res = step_c_register_data_path(&data);
    if (res) {
        STEP_C_ERR("register step_c data path fail = %d\n", res);
        goto exit_kfree;
    }
    icm406xx_sc_init_flag = 0;
    STEP_C_LOG("%s: OK\n", __func__);
    return 0;

exit_misc_device_register_failed:
    misc_deregister(&icm406xx_sc_device);
exit_create_attr_failed:
exit_init_failed:
exit_kfree:
exit:
    kfree(obj);
    obj = NULL;
    icm406xx_sc_init_flag =  -1;
    STEP_C_ERR("%s: err = %d\n", __func__, res);
    return res;
}

static int icm406xx_sc_attr_remove(void)
{
    int res = 0;

    res = icm406xx_sc_delete_attr(
        &(icm406xx_sc_init_info.platform_diver_addr->driver));
    if (res)
        STEP_C_ERR("icm406xx_sc_delete_attr fail: %d\n", res);
    else
        res = 0;
    res = misc_deregister(&icm406xx_sc_device);
    if (res)
        STEP_C_ERR("misc_deregister fail: %d\n", res);
    else
        res = 0;
    return res;
}

static int icm406xx_sc_remove(struct platform_device *pdev)
{
    STEP_C_LOG("icm406xx_sc_remove\n");
    return 0;
}

static int icm406xx_sc_probe(struct platform_device *pdev)
{
    STEP_C_LOG("icm406xx_sc_probe\n");
    pltfm_dev = pdev;
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id icm406xx_sc_of_match[] = {
    {.compatible = "invensense,icm406xx_sc",},
    {},
};
#endif

static struct platform_driver icm406xx_sc_driver = {
    .probe	  = icm406xx_sc_probe,
    .remove	 = icm406xx_sc_remove,
    .driver = {
        .name  = "icm406xx_sc",
#ifdef CONFIG_OF
        .of_match_table = icm406xx_sc_of_match,
#endif
    }
};

static int icm406xx_sc_local_remove(void)
{
    icm406xx_sc_attr_remove();
    return 0;
}

static int icm406xx_sc_local_init(void)
{
    if (platform_driver_register(&icm406xx_sc_driver))
        STEP_C_ERR("failed to register icm406xx_sc driver\n");
    icm406xx_sc_attr_create();
    if (-1 == icm406xx_sc_init_flag)		
        return -1;
    return 0;
}

static int __init icm406xx_sc_init(void)
{
    step_c_driver_add(&icm406xx_sc_init_info);
    return 0;
}

static void __exit icm406xx_sc_exit(void)
{
    /* nothing to do here */
}

module_init(icm406xx_sc_init);
module_exit(icm406xx_sc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icm406xx step counter driver");
