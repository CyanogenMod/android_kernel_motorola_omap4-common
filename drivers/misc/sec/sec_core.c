/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* Necessary includes for device drivers */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <asm/system.h>		/* cli(), *_flags */
#include <linux/uaccess.h>	/* copy_from/to_user */
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <mach/omap4-common.h>

#include "sec_core.h"
#include "ppa_call.h"
#include "mot_ppa.h"

ssize_t sec_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
int sec_open(struct inode *inode, struct file *filp);
int sec_release(struct inode *inode, struct file *filp);
static long sec_ioctl(struct file *file, unsigned int ioctl_num,
		unsigned long ioctl_param);
int sec_init(void);
void sec_exit(void);

#define AP_DIE_ID_BASE (0x4A002200)

/* Structure that declares the usual file */
/* access functions */
const struct file_operations sec_fops = {
	.read = sec_read,
	.open = sec_open,
	.release = sec_release,
	.unlocked_ioctl = sec_ioctl
};

/* Mapping of the module init and exit functions */
module_init(sec_init);
module_exit(sec_exit);

static struct miscdevice sec_dev = {
	MISC_DYNAMIC_MINOR,
	"sec",
	&sec_fops
};

static struct regulator *sec_efuse_regulator = NULL;


/******************************************************************************/
/*   KERNEL DRIVER APIs, ONLY IOCTL RESPONDS BACK TO USER SPACE REQUESTS      */
/******************************************************************************/
int sec_init(void)
{
	int result;

	result = misc_register(&sec_dev);

	if (result) {
		printk(KERN_ERR "sec: cannot obtain major number \n");
		return result;
	}

	printk(KERN_INFO "Inserting sec module\n");
	return 0;
}

void sec_exit(void)
{
	/* Freeing the major number */
	misc_deregister(&sec_dev);
}

int sec_open(struct inode *inode, struct file *filp)
{
	/* Not supported, return Success */
	return 0;
}

int sec_release(struct inode *inode, struct file *filp)
{

	/* Not supported, return Success */
	return 0;
}

ssize_t sec_read(struct file *filp, char *buf,
		 size_t count, loff_t *f_pos)
{
	/* Not supported, return Success */
	return 0;
}

ssize_t sec_write(struct file *filp, char *buf,
		  size_t count, loff_t *f_pos)
{
	/* Not supported, return Success */
	return 0;
}

long sec_ioctl(struct file *file, unsigned int ioctl_num,
		unsigned long ioctl_param)
{
	unsigned long count = 0;
	unsigned int die_id_address, buffer[4];
	int ret_val = SEC_KM_FAIL;

	SEC_EFUSE_PARM_T efuse_data;

	switch (ioctl_num) {

	case SEC_IOCTL_EFUSE_RAISE:

		if (sec_efuse_regulator == NULL)
			sec_efuse_regulator = regulator_get(NULL, "vfuse");

		if (sec_efuse_regulator == NULL)
			printk(KERN_ERR "Registration voltage regulator failed");
		else {
			regulator_set_voltage(sec_efuse_regulator, 1700000, 1700000);

			if (regulator_enable(sec_efuse_regulator) != 0)
				printk(KERN_ERR "Efuse voltage raising failed");
			else
				ret_val = SEC_KM_SUCCESS;

			mdelay(2);
		}

		break;

	case SEC_IOCTL_EFUSE_LOWER:

		if (sec_efuse_regulator == NULL)
			printk(KERN_ERR "No sec_efuse_regulator");
		else {
			if (regulator_disable(sec_efuse_regulator) != 0)
				printk(KERN_ERR "Efuse voltage lowering failed");
			else
				ret_val = SEC_KM_SUCCESS;
		}

		break;

	case SEC_IOCTL_READ_PROC_ID:

		/* Die Id for the Cortex Core */
		die_id_address = AP_DIE_ID_BASE;

		buffer[0] = omap_readl(die_id_address);
		buffer[1] = omap_readl(die_id_address+8);
		buffer[2] = omap_readl(die_id_address+12);
		buffer[3] = omap_readl(die_id_address+16);

		count = copy_to_user((void __user *) ioctl_param, (const void *) buffer, SEC_PROC_ID_SIZE);

		ret_val = SEC_KM_SUCCESS;

		break;

	case SEC_IOCTL_WRITE_FUSE:

		count = copy_from_user(&efuse_data, (void __user *)ioctl_param, sizeof(SEC_EFUSE_PARM_T));

		if (omap4_secure_dispatcher(API_HAL_MOT_EFUSE_WRITE, FLAG_START_HAL_CRITICAL, 2, efuse_data.which_bank, efuse_data.efuse_value, 0 , 0) == API_HAL_RET_VALUE_OK)
			ret_val = SEC_KM_SUCCESS;

		break;

	case SEC_IOCTL_READ_FUSE:

		count = copy_from_user(&efuse_data, (SEC_EFUSE_PARM_T *)ioctl_param, sizeof(SEC_EFUSE_PARM_T));

		if (count != 0)
			break;

		efuse_data.efuse_value = omap4_secure_dispatcher(API_HAL_MOT_EFUSE_READ, FLAG_START_HAL_CRITICAL, 1, efuse_data.which_bank, 0, 0, 0);

		count += copy_to_user((SEC_EFUSE_PARM_T *)ioctl_param, &efuse_data, sizeof(SEC_EFUSE_PARM_T));

		ret_val = SEC_KM_SUCCESS;
		break;

	default:
		printk(KERN_ERR "sec ioctl called with bad cmd : %d ", ioctl_num);
		break;
	}

	if (count != 0) {
		printk(KERN_ERR "sec ioctl operation %d failed, copy is 0x%lX\n", ioctl_num, count);
		ret_val = SEC_KM_FAIL;
	}

	return ret_val;
}

/******************************************************************************/
/*Kernel Module License Information                                           */
/******************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("MOTOROLA");
