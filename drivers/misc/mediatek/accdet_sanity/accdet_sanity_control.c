#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#define NO_DEVICE 0
#define HEADSET_MIC 1
#define HEADSET_NO_MIC 2
#define HEADSET_HOOK 3
extern int accdet_control_hs(int hs_state);

static ssize_t accdet_sanity_control_write_proc(struct file *file,
												const char __user *buf,
												size_t count, loff_t *offp)
{
	char *newbuf;
	int d;

	if(file == NULL ||buf == NULL || offp == NULL){
		printk(KERN_INFO "%s Pointer is null\n", __func__);
		goto fail;
	}

	if(*offp){
		printk(KERN_INFO "%s File position is not zero before write\n", __func__);
		goto fail;
	}

	newbuf = kmalloc(count + 1, GFP_KERNEL);
	if (0 == newbuf){
		printk(KERN_INFO "%s Memory allocation failed\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(newbuf, buf, count) != 0){
		printk(KERN_INFO "%s copy_from_user failed\n", __func__);
		goto end;
	}

	sscanf(newbuf, "%d", &d);

	switch(d){
		case NO_DEVICE:
			accdet_control_hs(NO_DEVICE);
			break;
		case HEADSET_MIC:
			accdet_control_hs(HEADSET_MIC);
			break;
		case HEADSET_NO_MIC:
			accdet_control_hs(HEADSET_NO_MIC);
			break;
		case HEADSET_HOOK:
			accdet_control_hs(HEADSET_HOOK);
			break;
		default:
			printk( KERN_INFO "accdet_write_proc : \n");
			break;
	}

end:
	kfree(newbuf);
	return count;

fail:
	return 0;
}

static const struct file_operations proc_accdet_sanity_control_operations = {
	.write		= accdet_sanity_control_write_proc
};

struct proc_dir_entry *proc_accdet_sanity_control_entry;

static int accdet_sanity_control_init(void)
{
	int ret = 0;
	proc_accdet_sanity_control_entry = proc_create_data("accdet_sanity_ctrl",
									S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
									NULL,
									&proc_accdet_sanity_control_operations,
									NULL);
	return ret;
}

static void accdet_sanity_control_exit(void)
{
	remove_proc_entry("accdet_sanity_ctrl", NULL);
}

module_init(accdet_sanity_control_init);
module_exit(accdet_sanity_control_exit);
