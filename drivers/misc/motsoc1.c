/*
 * Copyright (C) 2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/motsoc1.h>
#include <linux/wakelock.h>

#ifdef CONFIG_QUICK_WAKEUP
#include <linux/quickwakeup.h>
#include <linux/wakeup_timer_kernel.h>
#endif

#define NAME			     "motsoc1"

#define I2C_RETRY_DELAY			5
#define I2C_RETRIES			5
#define I2C_RESPONSE_LENGTH		3
#define MOTSOC1_MAXDATA_LENGTH		(1024*8)
#define MOTSOC1_DELAY_USEC		10
#define MOTSOC1_RESPONSE_MSG		0x02
#define MOTSOC1_RESPONSE_MSG_SUCCESS	0x00
#define MOTSOC1_OPCODE_LENGTH		3
#define MOTSOC1_RAM_ADDRESS_LENGTH      4
#define MOTSOC1_RAM_WRITE_BYTE_LENGTH   2
#define KDEBUG(format, s...)	if (g_debug)\
		pr_info(format, ##s)

static char g_debug;
#define CMD_BUF_SIZE (MOTSOC1_OPCODE_LENGTH + MOTSOC1_RAM_ADDRESS_LENGTH + \
	MOTSOC1_RAM_WRITE_BYTE_LENGTH + MOTSOC1_MAXDATA_LENGTH)
static unsigned char motsoc1_cmdbuff[CMD_BUF_SIZE];

struct motsoc1_data {
	struct i2c_client *client;
	struct motsoc1_platform_data *pdata;
	/* to avoid two i2c communications at the same time */
	struct mutex lock;
	int hw_initialized;
	atomic_t enabled;
	enum motsoc1_mode mode;
	int in_activity;
};

static int motsoc1_upgrade_status;

enum motsoc1_commands{
	FLASH_AD,
	FLASH_BYTE,
	FLASH_ERASE,
	FLASH_WR,
	RAM_CLR,
	FLASH_CHK,
	FLASH_SUM,
	CAM_START_AD,
	CAM_START,
	FLASH_SEL,
	GET_FLASH_ID,
	MOTSOC1_MEMORY_ACCESS,
	PROGRAM_CODE
};

#define MOTSOC1_CATEGORY_O_COMMAND 0x00
#define MOTSOC1_CATEGORY_F_COMMAND 0x0F

enum categoryF_opcode{
	FLASH_AD_OPCODE = 0x00,
	FLASH_BYTE_OPCODE = 0x04,
	FLASH_ERASE_OPCODE = 0x06,
	FLASH_WR_OPCODE = 0x07,
	RAM_CLR_OPCODE = 0x08,
	FLASH_CHK_OPCODE = 0x09,
	FLASH_SUM_OPCODE = 0x0A,
	CAM_START_AD_OPCODE = 0x0C,
	CAM_START_OPCODE = 0x12,
	GET_FLASH_ID_OPCODE = 0x3B
};

#define RAM_CLEAR_START 0x01
#define RAM_CLEARING 0x01

enum erase_method{
	SECTOR_ERASE = 0x01,
	CHIP_ERASE = 0x02,
	BLOCK_ERASE = 0x04
};

#define ERASING_STATUS_BYTE 0x01

enum erase_status{
	ERASE_COMPLETE,
	ERASING
};

#define SECTOR_PROGRAM_START_BYTE 0x01


enum categoryF_parameter_ram_access {
	INVALID = 0x00,
	READ_CATEGORY_PARAMETER = 0x01,
	WRITE_CATEGORY_PARAMETER = 0x02
};

#define MOTSOC1_MEMORY_ACCESS_OPCODE 0x00

enum motsoc1_memory_access {
	EIGHT_BIT_READ = 0x03,
	EIGHT_BIT_WRITE = 0x04,
	SIXTEEN_BIT_READ = 0x05,
	SIXTEEN_BIT_WRITE = 0x06,
	THIRTYTWO_BIT_READ = 0x07,
	THIRTYTWO_BIT_WRITE = 0x08
};

#define CHECK_SUM_STATUS_BYTE 0x01

enum checksum_sstatus {
	SUM_CHECK_COMPLETE,
	SUM_CHECKING
};

static unsigned char set_motsoc1_pins[3][22] = {
	{0x50, 0x00, 0x03, 0x00, 0x00, 0x10, 0x00, 0x00,
		0x00, 0xFF, 0xFF, 0xFE, 0xFF, 0x1F, 0x76,
		0x00, 0x18, 0x00, 0x00, 0x00, 0xFF, 0xFF},
	{0x50, 0x00, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x50, 0x00, 0x02, 0x00, 0x00, 0x10, 0x03, 0x00,
		0x00, 0xFF, 0xFF, 0xFE, 0xFF, 0x3F, 0x77,
		0x00, 0x20, 0x30, 0x00, 0x00, 0xFF, 0xFF}
};

enum versions {
	FIRMWARE_VERSION,
	PARAMETER_VERSION
};

enum version_byte {
	FIRMWARE_VERSION_BYTE = 0x02,
	PARAMETER_VERSION_BYTE = 0x06
};

#define VERSION_BYTE_LENGTH 2
#define REPEATED_CHECKING_MSG_LENGTH 5

enum flash_sum_bit {
	FLASH_CHK_B = 0x00,
	FLASH_CHK_H = 0x01,
	FLASH_CHK_16Mbit = 0x02
};

struct motsoc1_response {

	unsigned char len;
	unsigned char data;
};

enum motsoc1_error_code {
	EFF_COMM_BYTE_NUM_ERROR = 0xF0,
	COMMAND_CODE_ERROR = 0xF1,
	CATEGORY_CODE_ERROR = 0xF2,
	BYTE_NUMBER_ERROR = 0xF3,
	R_W_BYTES_LONG_ERROR = 0xF4,
	MODE_SWITCHED_ERROR = 0xFA,
	DURING_FLASH_WRITER_MODE_ERROR = 0xFF
};

struct motsoc1_data *motsoc1_misc_data;

static void motsoc1_device_turn_on_mclk(struct motsoc1_data *ps_motsoc1)
{
	KDEBUG("In motsoc1_device_turn_on_mclk\n");
	if (ps_motsoc1->pdata->mclk_on)
		ps_motsoc1->pdata->mclk_on();
}

static void motsoc1_device_turn_off_mclk(struct motsoc1_data *ps_motsoc1)
{
	KDEBUG("In motsoc1_device_turn_off_mclk\n");
	if (ps_motsoc1->pdata->mclk_off)
		ps_motsoc1->pdata->mclk_off();
}

static int motsoc1_i2c_write_read(struct motsoc1_data *ps_motsoc1, u8 *buf,
			int writelen, int readlen)
{
	int tries, err = 0;
	struct motsoc1_response *response;
	struct i2c_msg msgs[] = {
		{
			.addr = ps_motsoc1->client->addr,
			.flags = ps_motsoc1->client->flags,
			.len = writelen,
			.buf = buf,
		},
		{
			.addr = ps_motsoc1->client->addr,
			.flags = ps_motsoc1->client->flags | I2C_M_RD,
			.len = readlen,
			.buf = buf,
		},
	};
	if (buf == NULL || writelen == 0 || readlen == 0)
		return -EFAULT;

	if (ps_motsoc1->mode == FLASH_WRITER_MODE) {
		KDEBUG("MOTSOC1 In motsoc1_i2c_write_read\n");
		KDEBUG("MOTSOC1 sending: ");
		for (tries = 0; tries < writelen; tries++)
			KDEBUG("MOTSOC1 %02x", buf[tries]);
		KDEBUG("MOTSOC1 \n");
	}
	tries = 0;
	do {
		err = i2c_transfer(ps_motsoc1->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));
	if (err != 2) {
		dev_err(&ps_motsoc1->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
		KDEBUG("MOTSOC1 Read from ISP: ");
		for (tries = 0; tries < readlen; tries++)
			KDEBUG("MOTSOC1 %02x", buf[tries]);
		KDEBUG("MOTSOC1 \n");
		if (ps_motsoc1->mode == FLASH_WRITER_MODE) {
			response = (struct motsoc1_response *) buf;
			if ((response->data == EFF_COMM_BYTE_NUM_ERROR)
			|| (response->data == COMMAND_CODE_ERROR)
			|| (response->data == CATEGORY_CODE_ERROR)
			|| (response->data == BYTE_NUMBER_ERROR)
			|| (response->data == R_W_BYTES_LONG_ERROR)
			|| (response->data == MODE_SWITCHED_ERROR)
			|| (response->data == DURING_FLASH_WRITER_MODE_ERROR)) {
				pr_err("i2c cmd returned failure %02x\n",
					response->data);
				err = -EIO;
			}
		}
	}
	return err;
}

static int motsoc1_i2c_read(struct motsoc1_data *ps_motsoc1, u8 *buf, int len)
{
	int tries, err = 0;

	if (buf == NULL || len == 0)
		return -EFAULT;
	tries = 0;
	do {
		err = i2c_master_recv(ps_motsoc1->client, buf, len);
		if (err < 0)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err < 0) && (++tries < I2C_RETRIES));
	if (err < 0) {
		dev_err(&ps_motsoc1->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		KDEBUG("Read was successsful: \n");
		for (tries = 0; tries < err ; tries++)
			KDEBUG("MOTSOC1 %02x", buf[tries]);
		KDEBUG("MOTSOC1 \n");
	}
	return err;
}

static int motsoc1_i2c_write(struct motsoc1_data *ps_motsoc1, u8 * buf, int len)
{
	int err = 0;
	int tries = 0;

	KDEBUG(" Writing: \n");
	if (len <= 200)  {
		/* for long commands, print 1st 200 bytes */
		for (tries = 0; tries < len ; tries++)
			KDEBUG("MOTSOC1 %02x", buf[tries]);
	} else {
		for (tries = 0; tries < 20 ; tries++)
			KDEBUG("MOTSOC1 %02x", buf[tries]);
	}
	KDEBUG("total bytes = 0x%x\n", len);
	KDEBUG("MOTSOC1 \n");
	tries = 0;
	do {
		err = i2c_master_send(ps_motsoc1->client, buf, len);
		if (err < 0)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err < 0) && (++tries < I2C_RETRIES));

	if (err < 0) {
		dev_err(&ps_motsoc1->client->dev, "motsoc1: write error\n");
		err = -EIO;
	} else {
		KDEBUG("MOTSOC1 motsoc1 i2c write successful \n");
		err = 0;
	}
	return err;
}

static int motsoc1_i2c_write_read1(struct motsoc1_data *ps_motsoc1, u8 *buf,
			int writelen, int readlen)
{
	int err = 0;
	err = motsoc1_i2c_write(ps_motsoc1, buf, writelen);
	if (err < 0)
		KDEBUG("Unable to write \n");
		err = motsoc1_i2c_read(ps_motsoc1, buf, readlen);
		if (err < 0)
			KDEBUG("Unable to read \n");
	return err;
}

static int motsoc1_hw_init(struct motsoc1_data *ps_motsoc1)
{
	int err = 0;
	KDEBUG("MOTSOC1 in  motsoc1_hw_init\n");
	ps_motsoc1->hw_initialized = 1;
	return err;
}

static void motsoc1_device_power_off(struct motsoc1_data *ps_motsoc1)
{
	KDEBUG("MOTSOC1  in motsoc1_device_power_off\n");
	if (ps_motsoc1->hw_initialized == 1) {
		if (ps_motsoc1->pdata->power_off)
			ps_motsoc1->pdata->power_off();
		ps_motsoc1->hw_initialized = 0;
	}
}

static int motsoc1_device_power_on(struct motsoc1_data *ps_motsoc1)
{
	int err = 0;
	KDEBUG("In motsoc1_device_power_on\n");
	if (ps_motsoc1->pdata->power_on) {
		err = ps_motsoc1->pdata->power_on();
		if (err < 0) {
			dev_err(&ps_motsoc1->client->dev,
				"power_on failed: %d\n", err);
			goto out;
		}
	}
	if (!ps_motsoc1->hw_initialized) {
		err = motsoc1_hw_init(ps_motsoc1);
		if (err < 0) {
			motsoc1_device_power_off(ps_motsoc1);
			goto out;
		}
	}
out:
	return err;
}

static int motsoc1_enable(struct motsoc1_data *ps_motsoc1)
{
	int err = 0;
	KDEBUG("MOTSOC1 motsoc1_enable\n");
	if (!atomic_cmpxchg(&ps_motsoc1->enabled, 0, 1)) {
		err = motsoc1_device_power_on(ps_motsoc1);
		if (err < 0) {
			atomic_set(&ps_motsoc1->enabled, 0);
			goto out;
		}
	}
out:
	return err;
}

static int motsoc1_disable(struct motsoc1_data *ps_motsoc1)
{
	int err = 0;
	KDEBUG("MOTSOC1 motsoc1_disable\n");
	if (atomic_cmpxchg(&ps_motsoc1->enabled, 1, 0))
		motsoc1_device_power_off(ps_motsoc1);
	return err;
}

static int motsoc1_misc_open(struct inode *inode, struct file *file)
{
	int err = 0;
	KDEBUG("MOTSOC1 motsoc1_misc_open\n");
	KDEBUG("request and set GPIO\n");
	/* turn on MCLK */
	motsoc1_device_turn_on_mclk(motsoc1_misc_data);

	/* configure MOTSOC1 gpios */
	err = gpio_request(motsoc1_misc_data->pdata->gpio_reset,
		 "motsoc1 reset");
	if (err) {
		pr_err("MOTSOC1 getting reset gpio failed with %d\n", err);
		goto out;
	}
	gpio_direction_output(motsoc1_misc_data->pdata->gpio_reset, 0);

	err = nonseekable_open(inode, file);
	if (err < 0) {
		gpio_free(motsoc1_misc_data->pdata->gpio_reset);
		goto out;
	}
	file->private_data = motsoc1_misc_data;

	err = motsoc1_enable(motsoc1_misc_data);
out:
	return err;
}

static int motsoc1_misc_close(struct inode *inode, struct file *file)
{
	int err = 0;
	KDEBUG("MOTSOC1 motsoc1_misc_close\n");
	KDEBUG("release gpio 83\n");
	gpio_free(motsoc1_misc_data->pdata->gpio_reset);
	return err;
}


/* static char  num_of_8k = 0; */
static char  num_of_8k;

/* the caller function is resposible to free mem allocated in this function. */
void motsoc1_build_command(enum motsoc1_commands cmd, const char *inbuff,
	unsigned int *length, enum categoryF_parameter_ram_access \
	read_or_write, unsigned int byte_value)
{
	unsigned int index = 0, i, len = *length;

	switch (cmd) {
	case FLASH_AD:
		motsoc1_cmdbuff[index++] = 0x08; /* len  */
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = FLASH_AD_OPCODE;
		motsoc1_cmdbuff[index++] = (byte_value>>24) & 0xFF;
		motsoc1_cmdbuff[index++] = (byte_value>>16) & 0xFF;
		motsoc1_cmdbuff[index++] = (byte_value>>8) & 0xFF;
		motsoc1_cmdbuff[index++] = (byte_value & 0xFF);
		/* when resend flash rom address, reset num_of_8k,
		 * this is because
		 * send PROGRAM_CODE command needs to reset to 0x68000000
		 */
		num_of_8k = 0;
		break;
	case FLASH_ERASE:
		motsoc1_cmdbuff[index++] = 0x05; /* len  */
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = FLASH_ERASE_OPCODE; /* opcode */
		motsoc1_cmdbuff[index++] = byte_value;
		break;
	case RAM_CLR:
		motsoc1_cmdbuff[index++] = 0x05; /* len  */
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = RAM_CLR_OPCODE; /* opcode */
		motsoc1_cmdbuff[index++] = byte_value;
		break;
	case FLASH_CHK:
		motsoc1_cmdbuff[index++] = 0x05; /* len */
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = FLASH_CHK_OPCODE; /* opcode */
		motsoc1_cmdbuff[index++] = byte_value;
		break;
	case FLASH_SUM:
		motsoc1_cmdbuff[index++] = 0x05; /* len */
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = FLASH_SUM_OPCODE; /* opcode */
		motsoc1_cmdbuff[index++] = byte_value;
		break;
	case FLASH_WR:
		motsoc1_cmdbuff[index++] = 0x05;
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = FLASH_WR_OPCODE;
		motsoc1_cmdbuff[index++] = byte_value;
		break;
	case CAM_START_AD:
		motsoc1_cmdbuff[index++] = 0x08;
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = CAM_START_AD_OPCODE;
		motsoc1_cmdbuff[index++] = 0x10;
		motsoc1_cmdbuff[index++] = 0x00;
		motsoc1_cmdbuff[index++] = 0x00;
		motsoc1_cmdbuff[index++] = 0x00;
		break;
	case CAM_START:
		motsoc1_cmdbuff[index++] = 0x05;
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = CAM_START_OPCODE;
		motsoc1_cmdbuff[index++] = byte_value;
		break;
	case PROGRAM_CODE:
		/*code length */
		KDEBUG("MOTSOC1 No of bytes got from user = %d", len);
		motsoc1_cmdbuff[index++] = MOTSOC1_MEMORY_ACCESS_OPCODE;
		motsoc1_cmdbuff[index++] = EIGHT_BIT_WRITE;
		motsoc1_cmdbuff[index++] = 0x68;
		motsoc1_cmdbuff[index++] = 0x00;
		motsoc1_cmdbuff[index++] = 0x20*num_of_8k;
		if (num_of_8k++ == 8)
			num_of_8k = 0;
		motsoc1_cmdbuff[index++] = 0x00;
		motsoc1_cmdbuff[index++] = 0x20; /* MSB of 0x2000 - 8KB */
		motsoc1_cmdbuff[index++] = 0x00; /* LSB of 0x2000 - 8KB */

		/* copy data from user to kernel space */
		if (copy_from_user(motsoc1_cmdbuff+index, inbuff, len)) {
			pr_err("ISP copy from user returned error\n");
			index = 0;
		}
		index += MOTSOC1_MAX_PACKET_LENGTH;
		break;
	case FLASH_BYTE:
		motsoc1_cmdbuff[index++] = 0x06; /* len */
		motsoc1_cmdbuff[index++] = read_or_write;
		motsoc1_cmdbuff[index++] = MOTSOC1_CATEGORY_F_COMMAND;
		motsoc1_cmdbuff[index++] = FLASH_BYTE_OPCODE;
		if (byte_value < 0x101F0000) {
			motsoc1_cmdbuff[index++] = 0x00;
			motsoc1_cmdbuff[index++] = 0x00;
		} else {
			motsoc1_cmdbuff[index++] = 0x80;
			motsoc1_cmdbuff[index++] = 0x00;
		}
		break;
	case MOTSOC1_MEMORY_ACCESS:
		/* len LSB */
		motsoc1_cmdbuff[index++] = MOTSOC1_MEMORY_ACCESS_OPCODE;
		/* len MSB */
		motsoc1_cmdbuff[index++] = EIGHT_BIT_WRITE;
		for (i = 0; i < 22; i++)
			motsoc1_cmdbuff[index++] =
				set_motsoc1_pins[byte_value][i];
		break;
	default:
		pr_info("Invalid motsoc1 cmd \n");
		index = 0;
		break;
	}
	/*command length */
	*length = index;
}

static ssize_t motsoc1_misc_write(struct file *file, const char __user *buff,
				 size_t count,  loff_t *ppos)
{
	int  err = 0;
	struct motsoc1_data *ps_motsoc1;
	unsigned int len = (unsigned int)count;

	KDEBUG("ISP430 motsoc1_misc_write\n");
	ps_motsoc1 = motsoc1_misc_data;
	mutex_lock(&ps_motsoc1->lock);
	if ((len > MOTSOC1_MAXDATA_LENGTH) || (len == 0)) {
		pr_err("Error packet ize is more "
			"than MOTSOC1_MAXDATA_LENGTH or 0\n");
		err = -EINVAL;
		goto out;
	}
	KDEBUG("MOTSOC1 Got from user: ");
	KDEBUG("MOTSOC1 \n Leng = %d", len); /* debug */

	KDEBUG("MOTSOC1  motsoc1_misc_write: flash write mode\n");
	/* build the motsoc1 command to program code */
	motsoc1_build_command(PROGRAM_CODE, buff, &len, INVALID, 0);
	err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff, len);
	/* increment the current MSP write addr by count */
	if (err == 0) {
		/* return the number of bytes successfully written */
		err = len;
	}
out:
	mutex_unlock(&ps_motsoc1->lock);
	return err;
}

/* gpio toggling to switch modes(boot mode,normal mode)on MOTSOC1 */
void switch_motsoc1_mode(struct motsoc1_data *ps_motsoc1,
	enum motsoc1_mode mode)
{
	int err = 0;
	unsigned int cmdlen = 0;
	if (motsoc1_misc_data->mode == mode)
		return;

	if (mode == POWER_OFF_MODE) {
		KDEBUG("MOTSOC1 toggling to switch to power off mode\n");
		gpio_set_value(motsoc1_misc_data->pdata->gpio_reset, 0);
		udelay(MOTSOC1_DELAY_USEC);
		motsoc1_device_turn_off_mclk(ps_motsoc1);
		udelay(MOTSOC1_DELAY_USEC);
		mdelay(1);
	} else if (mode == FLASH_WRITER_MODE) {

		KDEBUG("MOTSOC1 toggling to switch to flash writer mode\n");
		/* Toggling reset pins to put into flash writer mode */
		motsoc1_device_turn_on_mclk(ps_motsoc1);
		gpio_set_value(motsoc1_misc_data->pdata->gpio_reset, 0);
		/* Delays necessary for ISP timing */
		mdelay(1);
		udelay(MOTSOC1_DELAY_USEC);
		gpio_set_value(motsoc1_misc_data->pdata->gpio_reset, 1);
		udelay(MOTSOC1_DELAY_USEC);
		mdelay(1);

	} else if (mode == PARAMETER_SETTING_MODE) {
		/*normal mode */
		KDEBUG("MOTSOC1 switch to parameter setting  mode\n");
		motsoc1_build_command(CAM_START_AD, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, 0x01);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);

		motsoc1_build_command(CAM_START, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, 0x01);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);
		mdelay(6);
	}
	motsoc1_misc_data->mode = mode;
}

static int motsoc1_get_version(struct motsoc1_data *ps_motsoc1, int versions)
{
	int err = 0;
	int version = 0, i;
	unsigned char temp_cmdbuff[REPEATED_CHECKING_MSG_LENGTH];

	KDEBUG("ISP Switch to flash Writer mode get version\n");
	switch_motsoc1_mode(ps_motsoc1, FLASH_WRITER_MODE);
	mdelay(10);
	KDEBUG("ISP Switch to parameter setting to get version\n");
	switch_motsoc1_mode(ps_motsoc1, PARAMETER_SETTING_MODE);
	mdelay(50);
	KDEBUG("MOTSOC1 firmware version: \n");
	motsoc1_cmdbuff[0] = 0x05;
	motsoc1_cmdbuff[1] = READ_CATEGORY_PARAMETER;
	motsoc1_cmdbuff[2] = MOTSOC1_CATEGORY_O_COMMAND;
	motsoc1_cmdbuff[3] = FIRMWARE_VERSION_BYTE;
	motsoc1_cmdbuff[4] = VERSION_BYTE_LENGTH;
	for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			temp_cmdbuff[i] = motsoc1_cmdbuff[i];
	while ((err = motsoc1_i2c_write_read1(ps_motsoc1,
		motsoc1_cmdbuff, 5, 3)) > 0) {
		if (motsoc1_cmdbuff[1] != MODE_SWITCHED_ERROR)
			break;
		for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			motsoc1_cmdbuff[i] = temp_cmdbuff[i];
		mdelay(50);
	}
	if (err >= 0) {
		version = (int)((motsoc1_cmdbuff[1] << 8)
			| (motsoc1_cmdbuff[2]));
		pr_err("ISP firmware version %02x %02x",
			motsoc1_cmdbuff[1], motsoc1_cmdbuff[2]);
	} else
		version = 0xFFFF;

	motsoc1_cmdbuff[0] = 0x05;
	motsoc1_cmdbuff[1] = READ_CATEGORY_PARAMETER;
	motsoc1_cmdbuff[2] = MOTSOC1_CATEGORY_O_COMMAND;
	motsoc1_cmdbuff[3] = PARAMETER_VERSION_BYTE;
	motsoc1_cmdbuff[4] = VERSION_BYTE_LENGTH;
	for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
		temp_cmdbuff[i] = motsoc1_cmdbuff[i];
	while ((err = motsoc1_i2c_write_read1(ps_motsoc1,
		motsoc1_cmdbuff, 5, 3)) > 0) {
		if (motsoc1_cmdbuff[1] != MODE_SWITCHED_ERROR)
			break;
		for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			motsoc1_cmdbuff[i] = temp_cmdbuff[i];
		mdelay(50);
	}
	if (err >= 0) {
		err = (int)(version | (motsoc1_cmdbuff[1] << 24)
			 | (motsoc1_cmdbuff[2]<<16));
		pr_err("ISP parameter version %02x %02x",
			motsoc1_cmdbuff[1], motsoc1_cmdbuff[2]);
	} else
		err = (int)(version | (0xFFFF << 16));

	pr_err("MOTSOC1 versions = 0x%x\n", err);
	return err;
}

static int motsoc1_misc_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int err = 0;
	struct motsoc1_data *ps_motsoc1 = file->private_data;
	unsigned int cmdlen = 0;
	int  i;
	unsigned int flash_rom_address;
	unsigned char temp_cmdbuff[REPEATED_CHECKING_MSG_LENGTH];

	mutex_lock(&ps_motsoc1->lock);

	pr_err("MOTSOC1 motsoc1_misc_ioctl = %d\n", cmd);
	switch (cmd) {
	case MOTSOC1_IOCTL_GET_FIRMWARE_VERSIONS:
		err = motsoc1_get_version(ps_motsoc1, FIRMWARE_VERSION);
		break;
	case MOTSOC1_IOCTL_GET_MODE:
		err = motsoc1_misc_data->mode;
		break;
	case MOTSOC1_IOCTL_FLASH_WRITER_MODE:
		KDEBUG("Setting flash writer mode!!!\n");
		switch_motsoc1_mode(ps_motsoc1, FLASH_WRITER_MODE);
		break;
	case MOTSOC1_IOCTL_POWER_OFF_MODE:
		KDEBUG("Setting power off  mode!!!\n");
		switch_motsoc1_mode(ps_motsoc1, POWER_OFF_MODE);
		break;
	case MOTSOC1_IOCTL_SET_PINS:
		/* set MOTSOC1 pins */
		for (i = 0; i < 3; i++) {
			motsoc1_build_command(MOTSOC1_MEMORY_ACCESS, NULL,
				&cmdlen, WRITE_CATEGORY_PARAMETER, i);
			err = motsoc1_i2c_write(ps_motsoc1,
				motsoc1_cmdbuff, cmdlen);
			if (err < 0)
				pr_err("Unable to set motsoc1 pins\n");
		}
		break;
	case MOTSOC1_IOCTL_CHIPERASE:
		KDEBUG("Chip Erase\n");
		if (copy_from_user(&flash_rom_address, argp,
			sizeof(flash_rom_address))) {
			pr_err("copy from user returned error\n");
			return -EFAULT;
		}
		KDEBUG("flash_rom_address = 0x%x\n", flash_rom_address);

		motsoc1_build_command(FLASH_AD, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, flash_rom_address);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);
		motsoc1_build_command(FLASH_ERASE, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, CHIP_ERASE);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);
		motsoc1_build_command(FLASH_ERASE, NULL, &cmdlen,
			 READ_CATEGORY_PARAMETER, ERASING_STATUS_BYTE);
		for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			temp_cmdbuff[i] = motsoc1_cmdbuff[i];
		while (!(err = motsoc1_i2c_write_read(ps_motsoc1,
			motsoc1_cmdbuff, cmdlen, I2C_RESPONSE_LENGTH))) {
			if (motsoc1_cmdbuff[1] == ERASE_COMPLETE)
				break;
			KDEBUG("Erase not done!\n");
			mdelay(50);
			 for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
				motsoc1_cmdbuff[i] = temp_cmdbuff[i];
		}

		break;
	case MOTSOC1_IOCTL_BLOCKERASE:
		KDEBUG("Block Erase \n");
		if (copy_from_user(&flash_rom_address, argp,
			sizeof(flash_rom_address))) {
			pr_err("copy from user returned error\n");
			return -EFAULT;
		}

		motsoc1_build_command(FLASH_AD, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, flash_rom_address);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);

		motsoc1_build_command(FLASH_ERASE, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, BLOCK_ERASE);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);
		motsoc1_build_command(FLASH_ERASE, NULL, &cmdlen,
			READ_CATEGORY_PARAMETER, ERASING_STATUS_BYTE);
		for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			temp_cmdbuff[i] = motsoc1_cmdbuff[i];
		while (!(err = motsoc1_i2c_write_read(ps_motsoc1,
			motsoc1_cmdbuff, cmdlen, I2C_RESPONSE_LENGTH))) {
			/* wait(10ms); */
			if (motsoc1_cmdbuff[1] == ERASE_COMPLETE)
				break;
			KDEBUG("Erase not done!\n");
			mdelay(50);
			for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
				motsoc1_cmdbuff[i] = temp_cmdbuff[i];
		}
		break;
	case MOTSOC1_IOCTL_SET_ROM_ADDRESS:
		/* for chip erase, we have to set flash_rom_address again */
		KDEBUG("Set Flash Byte \n");
		if (copy_from_user(&flash_rom_address, argp,
			sizeof(flash_rom_address))) {
			pr_err("copy from user returned error\n");
			return -EFAULT;
		}

		motsoc1_build_command(FLASH_AD, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, flash_rom_address);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);
		break;
	case MOTSOC1_IOCTL_SET_FLASH_BYTE:
		/* set program byte and clear motsoc1 internal ram */
		KDEBUG("Set Flash Byte \n");
		if (copy_from_user(&flash_rom_address,
			argp, sizeof(flash_rom_address))) {
			pr_err("copy from user returned error\n");
		return -EFAULT;
		}

		motsoc1_build_command(FLASH_BYTE, NULL,  &cmdlen,
			WRITE_CATEGORY_PARAMETER, flash_rom_address);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
			cmdlen);
		motsoc1_build_command(RAM_CLR, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, RAM_CLEAR_START);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
			cmdlen);
		motsoc1_build_command(RAM_CLR, NULL, &cmdlen,
			READ_CATEGORY_PARAMETER, RAM_CLEARING);
		for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			temp_cmdbuff[i] = motsoc1_cmdbuff[i];
		while (!(err = motsoc1_i2c_write_read(ps_motsoc1,
			motsoc1_cmdbuff, cmdlen, I2C_RESPONSE_LENGTH))) {
			if (motsoc1_cmdbuff[1] == 0x00)
				break;
			KDEBUG("Cleaning not done!\n");
			for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
				motsoc1_cmdbuff[i] = temp_cmdbuff[i];
		}
		break;
	case MOTSOC1_IOCTL_SEND_PROGRAMMING_FIRMWARE:
		KDEBUG("Send Programming Firmware\n");
		motsoc1_build_command(FLASH_WR, NULL,  &cmdlen,
			WRITE_CATEGORY_PARAMETER, SECTOR_PROGRAM_START_BYTE);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
			cmdlen);
		motsoc1_build_command(FLASH_WR, NULL, &cmdlen,
			READ_CATEGORY_PARAMETER, SECTOR_PROGRAM_START_BYTE);
		 for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			temp_cmdbuff[i] = motsoc1_cmdbuff[i];

		while (!(err = motsoc1_i2c_write_read(ps_motsoc1,
			motsoc1_cmdbuff, cmdlen, I2C_RESPONSE_LENGTH))) {
			if (motsoc1_cmdbuff[1] == 0x00)
				break;
			KDEBUG("Programming not done!\n");
			  for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
				motsoc1_cmdbuff[i] = temp_cmdbuff[i];
			mdelay(50);
		}
		break;
	case MOTSOC1_IOCTL_PARAMETER_SETTING_MODE:
		switch_motsoc1_mode(ps_motsoc1, PARAMETER_SETTING_MODE);
		break;

	case MOTSOC1_IOCTL_SET_DEBUG:
		/* enable or disble msp driver debug messages */
		if (copy_from_user(&g_debug, argp, sizeof(g_debug))) {
			KDEBUG("copy from user returned error\n");
			return -EFAULT;
		}
		break;
	case MOTSOC1_IOCTL_SET_UPGRADE_STATUS:
		KDEBUG("Set upgrade status\n");
		if (copy_from_user(&motsoc1_upgrade_status, argp,
			sizeof(motsoc1_upgrade_status))) {
			pr_err("copy from user upgrade status returned \
				error\n");
			return -EFAULT;
		}
		pr_err("MOTSOC1 status is %d\n", motsoc1_upgrade_status);
		break;
	case MOTSOC1_IOCTL_GET_UPGRADE_STATUS:
		KDEBUG("Get upgrade status\n");
		err = motsoc1_upgrade_status;
		break;
	case MOTSOC1_IOCTL_CHECK_SUM_ENTIRE_MEM:
		motsoc1_build_command(FLASH_CHK, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, 0x01<<FLASH_CHK_16Mbit);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);
		motsoc1_build_command(FLASH_CHK, NULL, &cmdlen,
			READ_CATEGORY_PARAMETER, CHECK_SUM_STATUS_BYTE);
		for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			temp_cmdbuff[i] = motsoc1_cmdbuff[i];

		while (!(err = motsoc1_i2c_write_read(ps_motsoc1,
			motsoc1_cmdbuff, cmdlen, I2C_RESPONSE_LENGTH))) {
			/* wait(10ms); */
			if (motsoc1_cmdbuff[1] == SUM_CHECK_COMPLETE)
				break;
			KDEBUG(" not done!\n");
			mdelay(50);
			for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
				motsoc1_cmdbuff[i] = temp_cmdbuff[i];
		}
		motsoc1_build_command(FLASH_SUM, NULL, &cmdlen,
			READ_CATEGORY_PARAMETER, 0x02);
		err = motsoc1_i2c_write_read1(ps_motsoc1, motsoc1_cmdbuff,
			cmdlen, 3);
		if (err >= 0) {
				err = (int)((motsoc1_cmdbuff[1] << 8)
			| (motsoc1_cmdbuff[2]));
		pr_err("ISP Whole Area Check SUM is %02x %02x",
			motsoc1_cmdbuff[1], motsoc1_cmdbuff[2]);
		}

		break;
	case MOTSOC1_IOCTL_CHECK_SUM_LAST_32KB:
		motsoc1_build_command(FLASH_BYTE, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, 0x101F8000);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
			cmdlen);
		motsoc1_build_command(FLASH_CHK, NULL, &cmdlen,
			WRITE_CATEGORY_PARAMETER, 0x01<<FLASH_CHK_H);
		err = motsoc1_i2c_write(ps_motsoc1, motsoc1_cmdbuff,
					cmdlen);

		motsoc1_build_command(FLASH_CHK, NULL, &cmdlen,
			READ_CATEGORY_PARAMETER, CHECK_SUM_STATUS_BYTE);
		for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
			temp_cmdbuff[i] = motsoc1_cmdbuff[i];

		while (!(err = motsoc1_i2c_write_read(ps_motsoc1,
			motsoc1_cmdbuff, cmdlen, I2C_RESPONSE_LENGTH))) {
			/* wait(10ms); */
			if (motsoc1_cmdbuff[1] == SUM_CHECK_COMPLETE)
				break;
			KDEBUG(" not done!\n");
			mdelay(50);
			for (i = 0; i < REPEATED_CHECKING_MSG_LENGTH; i++)
				motsoc1_cmdbuff[i] = temp_cmdbuff[i];
		}
		motsoc1_build_command(FLASH_SUM, NULL, &cmdlen,
			READ_CATEGORY_PARAMETER, 0x02);
		err = motsoc1_i2c_write_read1(ps_motsoc1, motsoc1_cmdbuff,
			cmdlen, 3);
		if (err >= 0) {
				err = (int)((motsoc1_cmdbuff[1] << 8)
			| (motsoc1_cmdbuff[2]));
		pr_err("ISP Whole Area Check SUM is %02x %02x",
			motsoc1_cmdbuff[1], motsoc1_cmdbuff[2]);
		}

		break;
	default:
		pr_err("MOTSOC1 MOTSOC1:Invalid ioctl command %d\n", cmd);
		err = -EINVAL;
	}

	mutex_unlock(&ps_motsoc1->lock);
	return err;
}

static const struct file_operations motsoc1_misc_fops = {
	.owner = THIS_MODULE,
	.open = motsoc1_misc_open,
	.ioctl = motsoc1_misc_ioctl,
	.write = motsoc1_misc_write,
	.release = motsoc1_misc_close,
};

static struct miscdevice motsoc1_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &motsoc1_misc_fops,
};

static int motsoc1_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct motsoc1_data *ps_motsoc1;
	int err = -1;
	dev_info(&client->dev, "motsoc1 probe begun\n");

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL, exiting\n");
		err = -ENODEV;
		goto err0;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}
	ps_motsoc1 = kzalloc(sizeof(*ps_motsoc1), GFP_KERNEL);
	if (ps_motsoc1 == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: %d\n", err);
		goto err0;
	}

	mutex_init(&ps_motsoc1->lock);
	ps_motsoc1->client = client;
	ps_motsoc1->mode = UNINITIALIZED;

	/* Set to passive mode by default */
	ps_motsoc1->in_activity = 0;
	g_debug = 0;

	ps_motsoc1->pdata = kmalloc(sizeof(*ps_motsoc1->pdata), GFP_KERNEL);
	if (ps_motsoc1->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err1;
	}
	memcpy(ps_motsoc1->pdata, client->dev.platform_data,
		sizeof(*ps_motsoc1->pdata));
	i2c_set_clientdata(client, ps_motsoc1);
	ps_motsoc1->client->flags &= 0x00;

	if (ps_motsoc1->pdata->init) {
		err = ps_motsoc1->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err2;
		}
	}

	err = motsoc1_device_power_on(ps_motsoc1);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err3;
	}
	atomic_set(&ps_motsoc1->enabled, 1);

	motsoc1_misc_data = ps_motsoc1;
	err = misc_register(&motsoc1_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "misc register failed: %d\n", err);
		goto err4;
	}

	motsoc1_device_power_off(ps_motsoc1);

	atomic_set(&ps_motsoc1->enabled, 0);

	dev_info(&client->dev, "motsoc1 probed\n");

	return 0;

err4:
	motsoc1_device_power_off(ps_motsoc1);
err3:
	if (ps_motsoc1->pdata->exit)
		ps_motsoc1->pdata->exit();
err2:
	kfree(ps_motsoc1->pdata);
err1:
	mutex_unlock(&ps_motsoc1->lock);
	mutex_destroy(&ps_motsoc1->lock);
	kfree(ps_motsoc1);
err0:
	return err;
}

static int __devexit motsoc1_remove(struct i2c_client *client)
{
	struct motsoc1_data *ps_motsoc1 = i2c_get_clientdata(client);
	pr_err("MOTSOC1 motsoc1_remove\n");
	misc_deregister(&motsoc1_misc_device);
	motsoc1_device_power_off(ps_motsoc1);
	if (ps_motsoc1->pdata->exit)
		ps_motsoc1->pdata->exit();
	kfree(ps_motsoc1->pdata);
	mutex_destroy(&ps_motsoc1->lock);
	kfree(ps_motsoc1);

	return 0;
}

static int motsoc1_resume(struct i2c_client *client)
{

	struct motsoc1_data *ps_motsoc1 = i2c_get_clientdata(client);
	int err = 0;
	KDEBUG("MOTSOC1 motsoc1_resume\n");
	mutex_lock(&ps_motsoc1->lock);

	if (motsoc1_enable(ps_motsoc1) < 0) {
		pr_err("motsoc1_resume failed\n");
		return err;
	}
	if (ps_motsoc1->mode != FLASH_WRITER_MODE)
		ps_motsoc1->mode = PARAMETER_SETTING_MODE;
	mutex_unlock(&ps_motsoc1->lock);
	return err;
}

static int motsoc1_suspend(struct i2c_client *client, pm_message_t mesg)
{

	struct motsoc1_data *ps_motsoc1 = i2c_get_clientdata(client);
	int err = 0;
	KDEBUG("MOTSOC1 motsoc1_suspend\n");
	mutex_lock(&ps_motsoc1->lock);
	if (ps_motsoc1->mode != FLASH_WRITER_MODE)
		ps_motsoc1->mode = PARAMETER_SETTING_MODE;
	if (motsoc1_disable(ps_motsoc1) < 0) {
		pr_err("motsoc1_suspend failed\n");
		goto out;
	}
out:
	mutex_unlock(&ps_motsoc1->lock);
	return err;
}

static const struct i2c_device_id motsoc1_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, motsoc1_id);

static struct i2c_driver motsoc1_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = motsoc1_probe,
	.remove = __devexit_p(motsoc1_remove),
	.resume = motsoc1_resume,
	.suspend = motsoc1_suspend,
	.id_table = motsoc1_id,
};

static int __init motsoc1_init(void)
{
	pr_debug("MOTSOC1 motsoc1_init\n");
	return i2c_add_driver(&motsoc1_driver);
}

static void __exit motsoc1_exit(void)
{
	KDEBUG("MOTSOC1 motsoc1_exit\n");
	i2c_del_driver(&motsoc1_driver);
	return;
}

module_init(motsoc1_init);
module_exit(motsoc1_exit);

MODULE_DESCRIPTION("MOTSOC1 sensor processor");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
