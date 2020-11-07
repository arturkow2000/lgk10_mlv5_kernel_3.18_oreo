#define pr_fmt(fmt) "[CCD] %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/upmu_common.h>
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>

//#define CYCLE_DEBUG

#define BATTERY_CYCLE_PATH "/vendor/persist-lg/battery/"
#define BATTERY_CYCLE_FILE BATTERY_CYCLE_PATH "cycle"
#define BATTERY_DELTA_FILE BATTERY_CYCLE_PATH "delta"
#define BATTERY_OFFSET_FILE BATTERY_CYCLE_PATH "soc_offset"
#define BATTERY_FILE_LENGTH 10

#define BATTERY_CYCLE_DELTA_MAX 200

static DEFINE_MUTEX(update_mutex);

static int g_battery_cycle_count = 0;
static int g_battery_cycle_delta = 0;
static int g_battery_cycle_offset = 0;

static int g_battery_cycle_offset_cbc = 0;	/* offset used for cbc */

static bool cycle_enabled(void)
{
	if (get_boot_mode() == NORMAL_BOOT)
		return true;
	if (get_boot_mode() == CHARGER_BOOT)
		return true;

	return false;
}

static int cycle_read_from_fs(char *filename)
{
	mm_segment_t old_fs = get_fs();
	char temp_str[BATTERY_FILE_LENGTH];
	int fd = 0;
	int result = 0;
	int value_read = -EINVAL;

	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		pr_err("open error %s (%d) \n", filename, fd);
		goto Error;
	}
	memset(temp_str, 0x00, sizeof(temp_str));
	result = sys_read(fd, temp_str, sizeof(temp_str));
	if (result < 0) {
		pr_err("read error %s (%d)\n", filename, result);
		goto Error;
	}
	result = kstrtoint(temp_str, 10, &value_read);
	if (result != 0) {
		pr_err("kstrtoint Error\n");
		goto Error;
	}

Error:
	sys_close(fd);
	set_fs(old_fs);
	return value_read;
}

static int cycle_write_to_fs(char *filename, int value)
{
	mm_segment_t old_fs = get_fs();
	char temp_str[BATTERY_FILE_LENGTH];
	int fd = 0;
	int result = 0;
	int value_write = -EINVAL;
	size_t size;

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY | O_CREAT | O_TRUNC | S_IROTH, 0664);
	if (fd < 0) {
		pr_err("open error %s (%d)\n", filename, fd);
		goto Error;
	}

	memset(temp_str, 0x00, sizeof(temp_str));
	size = snprintf(temp_str, sizeof(temp_str), "%d\n", value);
	result = sys_write(fd, temp_str, size);

	if (result < 0) {
		pr_err("write error %s (%d) \n", filename, result);
		goto Error;
	}
	value_write = result;
	sys_fsync(fd);

Error:
	sys_close(fd);
	sys_chmod(filename, 0664);
	set_fs(old_fs);
	return value_write;
}

int get_battery_cycle_count(void)
{
	return g_battery_cycle_count;
}
EXPORT_SYMBOL(get_battery_cycle_count);

int set_battery_cycle_count(int cycle)
{
	/* cycle should be increased */
	if (cycle <= g_battery_cycle_count)
		return 0;

	mutex_lock(&update_mutex);

	g_battery_cycle_count = cycle;

	cycle_write_to_fs(BATTERY_CYCLE_FILE, g_battery_cycle_count);

	mutex_unlock(&update_mutex);

	pr_info("cycle changed to %d\n", cycle);

	return 0;
}
EXPORT_SYMBOL(set_battery_cycle_count);

int get_battery_cycle_offset(void)
{
	return g_battery_cycle_offset_cbc;
}
EXPORT_SYMBOL(get_battery_cycle_offset);

int set_battery_cycle_offset(int offset)
{
	if (!cycle_enabled())
		return 0;

	mutex_lock(&update_mutex);

	g_battery_cycle_offset = offset;

	cycle_write_to_fs(BATTERY_OFFSET_FILE, offset);

	mutex_unlock(&update_mutex);

	pr_info("offset changed to %d\n", g_battery_cycle_offset);

	return 0;
}
EXPORT_SYMBOL(set_battery_cycle_offset);

void battery_cycle_init(void)
{
	bool clear_cycle = false;
	int cycle_count;
	int cycle_delta;
	int cycle_offset;

	if (!cycle_enabled())
		return;

	cycle_count = cycle_read_from_fs(BATTERY_CYCLE_FILE);
	cycle_delta = cycle_read_from_fs(BATTERY_DELTA_FILE);
	cycle_offset = cycle_read_from_fs(BATTERY_OFFSET_FILE);

	if (cycle_count < 0) {
		pr_err("failed to read count\n");
		clear_cycle = true;
	}
	if (cycle_delta < 0) {
		pr_err("failed to read delta\n");
		clear_cycle = true;
	}
	if (cycle_offset < 0) {
		pr_err("failed to read offset\n");
		clear_cycle = true;
	}

	if (is_battery_remove_pmic() == 1) {
		pr_warn("battery removed\n");
		clear_cycle = true;
	}

	if (clear_cycle) {
		pr_info("clear battery cycle\n");

		mutex_lock(&update_mutex);

		cycle_write_to_fs(BATTERY_CYCLE_FILE, 0);
		cycle_write_to_fs(BATTERY_DELTA_FILE, 0);
		cycle_write_to_fs(BATTERY_OFFSET_FILE, 0);

		mutex_unlock(&update_mutex);

		cycle_count = 0;
		cycle_delta = 0;
		cycle_offset = 0;
	}

	g_battery_cycle_count = cycle_count;
	g_battery_cycle_delta = cycle_delta;
	g_battery_cycle_offset = cycle_offset;
	g_battery_cycle_offset_cbc = cycle_offset;

	pr_info("battery cycle = %d.%02d offset = %d\n",
			g_battery_cycle_count,
			g_battery_cycle_delta,
			g_battery_cycle_offset);
}
EXPORT_SYMBOL(battery_cycle_init);

void battery_cycle_update(int soc)
{
	static int pre_soc = -EINVAL;
	int delta_increased = 0;
	int cycle_increased = 0;
	int delta;
	bool update_to_fs = false;

	if (!cycle_enabled())
		return;

	if (pre_soc == -EINVAL)
		pre_soc = soc;

	delta_increased = abs(pre_soc - soc);
#ifdef CYCLE_DEBUG
	delta_increased += 138;
#endif
	/* soc changed. update to fs */
	if (delta_increased) {
		update_to_fs = true;
		pre_soc = soc;
	}

	/* no need to update */
	if (!update_to_fs)
		return;

	delta = g_battery_cycle_delta + delta_increased;
	cycle_increased += (delta / BATTERY_CYCLE_DELTA_MAX);

	mutex_lock(&update_mutex);

	if (cycle_increased) {
		g_battery_cycle_count += cycle_increased;
		delta %= BATTERY_CYCLE_DELTA_MAX;

		cycle_write_to_fs(BATTERY_CYCLE_FILE,
				g_battery_cycle_count);
	}

	g_battery_cycle_delta = delta;

	cycle_write_to_fs(BATTERY_DELTA_FILE, g_battery_cycle_delta);

	mutex_unlock(&update_mutex);

	pr_info("battery cycle = %d.%02d\n",
			g_battery_cycle_count,
			g_battery_cycle_delta);
}
EXPORT_SYMBOL(battery_cycle_update);
