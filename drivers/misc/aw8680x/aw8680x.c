/*
 * aw8680x.c   aw8680x sensor module
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 *  Author: ZhangPengBiao <zhangpengbiao@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include "aw_protocol_config.h"
#include "aw_protocol_map.h"
#include "aw_com_protocol.h"
#include "aw_protocol_fun.h"
#include "aw_soc_protocol_interface.h"
#include "aw_bin_parse.h"
#include "aw8680x.h"
/******************************************************
*
* Marco
*
******************************************************/
#define AW8680X_I2C_NAME "aw8680x_sensor"
#define AW8680X_DRIVER_VERSION "v1.0.0"
#define AW8680X_I2C_RETRIES 3
#define AW8680X_BIN_INIT_DELAY 5000
#define AW8680X_ADB_BIN_INIT_DELAY 20

struct aw8680x *g_aw8680x;
/***********************************************************
 * The name of the bin file that needs to be
 * obtained from /system/vendor/firmware
************************************************************/
static char *aw8680x_flash_app_bin = "aw8680x_flash_app.bin";
static char *aw8680x_flash_boot_bin = "aw8680x_flash_boot.bin";
static char *aw8680x_sram_bin = "aw8680x_sram.bin";

/******************************************************
 *
 * aw8680x i2c IO
 *
 ******************************************************/
static int32_t aw8680x_soc_i2c_writes(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	struct i2c_msg msgs[] = {
		{
			.addr = p_aw8680x->i2c->addr,
			.flags = I2C_WRITE_FLAG,
			.len = p_aw8680x->p_gui_data_s.soc_data_len,
			.buf = p_aw8680x->p_protocol_tx_data,
		},
	};

	ret = i2c_transfer(p_aw8680x->i2c->adapter, msgs, ONE_MSG_NUM);
	if (ret < DATA_INIT)
		AWLOGE(p_aw8680x->dev, "soc i2c write error: %d", ret);

	return ret;
}

static int32_t aw8680x_soc_i2c_reads(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	struct i2c_msg msgs[] = {
		{
			.addr = p_aw8680x->i2c->addr,
			.flags = I2C_READ_FLAG,
			.len = p_aw8680x->p_gui_data_s.ui_rd_data_len,
			.buf = p_aw8680x->p_protocol_rx_data,
		},
	};

	ret = i2c_transfer(p_aw8680x->i2c->adapter, msgs, ONE_MSG_NUM);
	if (ret < DATA_INIT)
		AWLOGE(p_aw8680x->dev, "soc i2c read error: %d", ret);

	return ret;
}

static int32_t aw8680x_register_i2c_reads(struct aw8680x *p_aw8680x,
						uint8_t reg_addr, uint32_t len)
{
	int32_t ret = DATA_INIT;

	struct i2c_msg msgs[] = {
		{
			.addr = p_aw8680x->i2c->addr,
			.flags = I2C_WRITE_FLAG,
			.len = 1,
			.buf = &reg_addr,
		},
		{
			.addr = p_aw8680x->i2c->addr,
			.flags = I2C_READ_FLAG,
			.len = len,
			.buf = p_aw8680x->read_data,
		},
	};

	ret = i2c_transfer(p_aw8680x->i2c->adapter, msgs, TWO_MSG_NUM);
	if (ret < DATA_INIT)
		AWLOGE(p_aw8680x->dev, "register i2c read error: %d", ret);

	return ret;
}

static int32_t aw8680x_register_i2c_writes(struct aw8680x *p_aw8680x,
				uint8_t reg_addr, uint8_t *buf, uint32_t len)
{
	int32_t ret = DATA_INIT;
	uint8_t *data = NULL;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		AWLOGE(p_aw8680x->dev, "can not allocate memory");
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(p_aw8680x->i2c, data, len + 1);
	if (ret < DATA_INIT)
		AWLOGE(p_aw8680x->dev, "register i2c write error: %d", ret);
	kfree(data);

	return ret;
}

static int32_t aw8680x_get_key_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	ret = aw8680x_register_i2c_reads(p_aw8680x, KEY_DATA_ADDR, 1);
	if (ret < DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "failed to read KEY_DATA_ADDR : %d",
									ret);
		return -EIO;
	}
	p_aw8680x->info.key_data = p_aw8680x->read_data[0];
	AWLOGI(p_aw8680x->dev, "key data = 0x%x", p_aw8680x->info.key_data);

	return AW_SUCCESS;
}

int32_t aw8680x_get_adc_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;

	ret = aw8680x_register_i2c_reads(p_aw8680x, ADC_DATA_ADDR,
								ADC_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "failed to read adc data, ret is : %d",
									ret);
		return -EIO;
	}
	memcpy(&(p_aw8680x->info.adc_data), p_aw8680x->read_data, ADC_DATA_LEN);
	for (i = DATA_INIT; i < ADC_DATA_NUM; i++) {
		AWLOGI(p_aw8680x->dev, "adc data[%d] = 0x%x", i,
						p_aw8680x->info.adc_data[i]);
	}

	return AW_SUCCESS;
}

int32_t aw8680x_get_raw_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;

	ret = aw8680x_register_i2c_reads(p_aw8680x, RAW_DATA_ADDR,
								RAW_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "failed to read raw data, ret is : %d",
									ret);
		return -EIO;
	}
	memcpy(&(p_aw8680x->info.raw_data), p_aw8680x->read_data, RAW_DATA_LEN);
	for (i = DATA_INIT; i < RAW_DATA_NUM; i++) {
		AWLOGI(p_aw8680x->dev, "raw data[%d] = 0x%x", i,
						p_aw8680x->info.raw_data[i]);
	}

	return AW_SUCCESS;
}

int32_t aw8680x_get_force_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;

	ret = aw8680x_register_i2c_reads(p_aw8680x, FORCE_DATA_ADDR,
								FORCE_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "failed to read force data, ret is : %d",
									ret);
		return -EIO;
	}
	memcpy(&(p_aw8680x->info.force_data), p_aw8680x->read_data,
								FORCE_DATA_LEN);
	for (i = DATA_INIT; i < FORCE_DATA_NUM; i++) {
		AWLOGI(p_aw8680x->dev, "force data[%d] = 0x%x", i,
						p_aw8680x->info.force_data[i]);
	}

	return AW_SUCCESS;
}

int32_t aw8680x_get_base_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;

	ret = aw8680x_register_i2c_reads(p_aw8680x, BASE_DATA_ADDR,
								BASE_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "failed to read base data, ret is : %d",
									ret);
		return -EIO;
	}
	memcpy(&(p_aw8680x->info.base_data), p_aw8680x->read_data,
								BASE_DATA_LEN);
	for (i = DATA_INIT; i < BASE_DATA_NUM; i++) {
		AWLOGI(p_aw8680x->dev, "base data[%d] = 0x%x", i,
						p_aw8680x->info.base_data[i]);
	}

	return AW_SUCCESS;
}

int32_t aw8680x_get_diff_data(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t i = DATA_INIT;

	ret = aw8680x_register_i2c_reads(p_aw8680x, DIFF_DATA_ADDR,
								DIFF_DATA_LEN);
	if (ret < DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "failed to read diff data, ret is : %d",
									ret);
		return -EIO;
	}
	memcpy(&(p_aw8680x->info.diff_data), p_aw8680x->read_data,
								DIFF_DATA_LEN);
	for (i = 0; i < DIFF_DATA_NUM; i++)
		AWLOGI(p_aw8680x->dev, "diff data[%d] = 0x%x", i,
						p_aw8680x->info.diff_data[i]);

	return AW_SUCCESS;
}

static void aw8680x_wake_pin(struct aw8680x *p_aw8680x)
{
	AWLOGI(p_aw8680x->dev, "enter");

	if (p_aw8680x->wake_flag == false) {
		AWLOGE(p_aw8680x->dev, "wake gpio is err!");
		return;
	}
	gpio_set_value_cansleep(p_aw8680x->wake_gpio, HIGH_LEVEL);
	usleep_range(1000, 1100);
	gpio_set_value_cansleep(p_aw8680x->wake_gpio, LOW_LEVEL);
}

static void aw8680x_active_mode_set(struct aw8680x *p_aw8680x)
{
	uint8_t reg_val = HIGH_SPEED;

	AWLOGI(p_aw8680x->dev, "enter");

	aw8680x_wake_pin(p_aw8680x);
	aw8680x_register_i2c_writes(p_aw8680x, REG_SCAN_MODE_SWITCH_ADDR,
						&reg_val, REGVAL_LEN_ONE_byte);
}

static void aw8680x_idle_mode_set(struct aw8680x *p_aw8680x)
{
	uint8_t reg_val = LOW_SPEED;

	AWLOGI(p_aw8680x->dev, "enter");

	aw8680x_wake_pin(p_aw8680x);
	aw8680x_register_i2c_writes(p_aw8680x, REG_SCAN_MODE_SWITCH_ADDR,
						&reg_val, REGVAL_LEN_ONE_byte);
}

static void aw8680x_sleep_mode_set(struct aw8680x *p_aw8680x)
{
	uint8_t reg_val = LOW_SPEED;

	AWLOGI(p_aw8680x->dev, "enter");

	aw8680x_wake_pin(p_aw8680x);
	aw8680x_register_i2c_writes(p_aw8680x, REG_SCAN_MODE_SWITCH_ADDR,
						&reg_val, REGVAL_LEN_ONE_byte);
}

static void aw8680x_set_common_info(struct aw8680x *p_aw8680x,
						unsigned short soc_data_len,
						unsigned char module_id,
						unsigned char event_id)
{
	p_aw8680x->p_gui_data_s.soc_data_len = soc_data_len;
	p_aw8680x->p_gui_data_s.module_id = module_id; /* module id */
	p_aw8680x->p_gui_data_s.event_id = event_id; /* evnet id */
	p_aw8680x->module_id = module_id;
}

/* pack data and i2c_write data to mcu */
static int32_t aw8680x_send(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	/* Pack instruction data according to soc protocol */
	ret = aw_soc_protocol_pack_interface(&(p_aw8680x->p_gui_data_s),
						p_aw8680x->p_protocol_tx_data);
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "soc data update pack fail!");
		return ret;
	}
	/* Send the packaged data to the MCU through the i2c write interface */
	ret = aw8680x_soc_i2c_writes(p_aw8680x);
	if (ret < DATA_INIT)
		return ret;

	return AW_SUCCESS;
}

/* i2c_read ack_data from IC and unpack ack_data */
static int32_t aw8680x_ack(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	/* Read the information processed by the instruction
	 * through the i2c read interface
	 */
	ret = aw8680x_soc_i2c_reads(p_aw8680x);
	if (ret < DATA_INIT)
		return ret;
	/* Unpack instruction data according to soc protocol */
	ret = aw_soc_protocol_unpack_interface(&(p_aw8680x->p_gui_data_s),
						p_aw8680x->p_protocol_rx_data);
	if ((p_aw8680x->p_gui_data_s.module_id == p_aw8680x->module_id) &&
			(ret == DATA_INIT) &&
			(p_aw8680x->p_gui_data_s.err_flag == DATA_INIT) &&
			(p_aw8680x->p_protocol_rx_data[7] == 1))
		return AW_SUCCESS;

	return -ACK_ERR;
}


/* Get information in ack data area */
static void aw8680x_get_info(struct aw8680x *p_aw8680x)
{
	/* connect */
	if (p_aw8680x->p_gui_data_s.module_id == HANDSHAKE_ID) {
		if (p_aw8680x->p_gui_data_s.event_id == CONNECT_ACK_ID)
			p_aw8680x->pc_location =
				p_aw8680x->p_gui_data_s.soc_data[0] |
				(p_aw8680x->p_gui_data_s.soc_data[1] << 8) |
				(p_aw8680x->p_gui_data_s.soc_data[2] << 16) |
				(p_aw8680x->p_gui_data_s.soc_data[3] << 24);
		else if (p_aw8680x->p_gui_data_s.event_id == CHIP_ID_ACK)
			p_aw8680x->chipid_data =
				p_aw8680x->p_gui_data_s.soc_data[0] |
				(p_aw8680x->p_gui_data_s.soc_data[1] << 8) |
				(p_aw8680x->p_gui_data_s.soc_data[2] << 16) |
				(p_aw8680x->p_gui_data_s.soc_data[3] << 24);
		else if (p_aw8680x->p_gui_data_s.event_id == VERSION_ACK_ID)
			p_aw8680x->flash_boot_version_in_soc =
				p_aw8680x->p_gui_data_s.soc_data[0] |
				(p_aw8680x->p_gui_data_s.soc_data[1] << 8) |
				(p_aw8680x->p_gui_data_s.soc_data[2] << 16) |
				(p_aw8680x->p_gui_data_s.soc_data[3] << 24);
	}
	/* flash read */
	if ((p_aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == FLASH_READ_ACK_ID))
		memcpy(p_aw8680x->read_data, p_aw8680x->p_gui_data_s.soc_data,
							p_aw8680x->read_len);
}



static void aw8680x_send_delay(struct aw8680x *p_aw8680x)
{
	/* erase sector delay */
	if ((p_aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == FLASH_ERASE_ID)) {
		mdelay(300);
	/* write flash delay */
	} else if ((p_aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == FLASH_WRITE_ID)) {
		mdelay(2);
	/* write sram delay */
	} else if ((p_aw8680x->p_gui_data_s.module_id == RAM_ID) &&
		(p_aw8680x->p_gui_data_s.event_id == RAM_WRITE_ID)) {
		mdelay(2);
	/* other delay */
	} else {
		udelay(80);
	}
}

static void aw8680x_ack_delay(struct aw8680x *p_aw8680x)
{
	int32_t delay_count = DATA_INIT;

	/* jump delay */
	if (p_aw8680x->p_gui_data_s.module_id == END_ID)
		mdelay(JUMP_INIT_TIME);
	/* Flash option failure handling mechanism */
	while ((p_aw8680x->p_gui_data_s.module_id == FLASH_ID)
		&& (p_aw8680x->p_gui_data_s.event_id == FLASH_ERASE_ACK_ID)
		&& (p_aw8680x->ack_flag != 0)) {
		mdelay(5);
		delay_count += 1;
		p_aw8680x->ack_flag = aw8680x_ack(p_aw8680x);
		if (delay_count == 5)
			break;
	}
}

static int32_t aw8680x_soc_protocol_set(struct aw8680x *p_aw8680x,
				    unsigned int addr, unsigned short read_len)
{
	int32_t ret = DATA_INIT;

	/* Configure some command information */
	p_aw8680x->p_gui_data_s.addr = addr; /* Read or write addr */
	p_aw8680x->p_gui_data_s.read_len = read_len; /* Read addr data length */

	ret = aw8680x_send(p_aw8680x);
	if (ret != DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "failed to send, ret is : %d", ret);
		return ret;
	}
	aw8680x_send_delay(p_aw8680x);
	/* Flash erase failure handling mechanism */
	p_aw8680x->ack_flag = aw8680x_ack(p_aw8680x);
	aw8680x_ack_delay(p_aw8680x);
	if (p_aw8680x->ack_flag != DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "ack flag = %d", p_aw8680x->ack_flag);
		return p_aw8680x->ack_flag;
	}
	/* get ack information */
	aw8680x_get_info(p_aw8680x);

	return AW_SUCCESS;
}

static int32_t aw8680x_connect(struct aw8680x *p_aw8680x)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, HANDSHAKE_ID, CONNECT_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, SOC_ADDR, SOC_READ_LEN);
}

static int32_t aw8680x_flash_boot_version_in_soc_get(struct aw8680x *p_aw8680x)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, HANDSHAKE_ID,
								VERSION_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, SOC_ADDR, SOC_READ_LEN);
}

static int32_t aw8680x_flash_app_version_in_soc_get(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	ret = aw8680x_connect(p_aw8680x);
	if (ret != AW_SUCCESS) {
		if (ret == FLASH_APP_IN_SOC_BLANK) {
			AWLOGI(p_aw8680x->dev, "the flash app in soc is blank");
		} else {
			AWLOGE(p_aw8680x->dev, "get flash app version failed, ret is :%d", ret);
		}
	}

	p_aw8680x->flash_app_version_in_soc = p_aw8680x->pc_location;

	return ret;
}

static int32_t
aw8680x_jump_flash_app(struct aw8680x *p_aw8680x, uint32_t jump_addr)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, END_ID, FLASH_JUMP_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, jump_addr, SOC_READ_LEN);
}

static int32_t aw8680x_jump_sram(struct aw8680x *p_aw8680x, uint32_t jump_addr)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, END_ID, RAM_JUMP_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, jump_addr, SOC_READ_LEN);
}

static int32_t aw8680x_erase_sector(struct aw8680x *p_aw8680x, uint32_t addr,
							uint32_t sector_num)
{
	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, FLASH_ID,
								FLASH_ERASE_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, addr, sector_num);
}

static void aw8680x_hw_reset(struct aw8680x *p_aw8680x)
{
	AWLOGI(p_aw8680x->dev, "enter");

	gpio_set_value_cansleep(p_aw8680x->reset_gpio, HIGH_LEVEL);
	usleep_range(150, 200);
	gpio_set_value_cansleep(p_aw8680x->reset_gpio, LOW_LEVEL);
	usleep_range(3000, 5000);
}

static void aw8680x_stay_boot(struct aw8680x *p_aw8680x)
{
	uint8_t reg_val = AW8680X_STAY_UBOOT;

	aw8680x_register_i2c_writes(p_aw8680x, REG_STAY_UBOOT,
						&reg_val, REGVAL_LEN_ONE_byte);
	usleep_range(10000, 15000);
}

static int32_t aw8680x_jump_boot(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t connect_count = CONNECT_RETRY_TIME;

	while (connect_count--) {
		aw8680x_hw_reset(p_aw8680x);
		aw8680x_stay_boot(p_aw8680x);
		ret = aw8680x_connect(p_aw8680x);
		if (ret == AW_SUCCESS) {
			if ((p_aw8680x->pc_location == PC_POINT_ROM_BOOT) ||
				(p_aw8680x->pc_location == PC_POINT_FLASH_BOOT))
			break;
		}
	}
	if ((p_aw8680x->pc_location == PC_POINT_ROM_BOOT) ||
			(p_aw8680x->pc_location == PC_POINT_FLASH_BOOT)) {
		AWLOGI(p_aw8680x->dev, "jump boot success!");
		return AW_SUCCESS;
	}
	AWLOGE(p_aw8680x->dev, "jump boot fail! ret is %d", ret);

	return -ERR_FLAG;
}

static int32_t aw8680x_flash_cycle_write(struct aw8680x *p_aw8680x,
			int cycle_num, int effect_len, int32_t flash_flag)
{
	uint32_t valid_data_addr = DATA_INIT;
	uint32_t download_addr = DATA_INIT;
	uint32_t valid_addr_offset = DATA_INIT;
	uint32_t download_addr_offset = DATA_INIT;

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
		valid_data_addr =
		p_aw8680x->flash_boot_bin->header_info[0].valid_data_addr;
		download_addr =
			p_aw8680x->flash_boot_bin->header_info[0].download_addr;
	} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
		valid_data_addr =
		p_aw8680x->flash_app_bin->header_info[0].valid_data_addr;
		download_addr =
			p_aw8680x->flash_app_bin->header_info[0].download_addr;
	}
	valid_addr_offset = valid_data_addr + cycle_num * WRITE_FLASH_MAX;
	download_addr_offset = download_addr + WRITE_FLASH_MAX * cycle_num;
	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
		memcpy(p_aw8680x->p_gui_data_s.soc_data, &(p_aw8680x->
				flash_boot_bin->info.data[valid_addr_offset]),
			effect_len);
	} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
		memcpy(p_aw8680x->p_gui_data_s.soc_data, &(p_aw8680x->
				flash_app_bin->info.data[valid_addr_offset]),
			effect_len);
	}
	aw8680x_set_common_info(p_aw8680x, effect_len, FLASH_ID, FLASH_WRITE_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, download_addr_offset,
								SOC_READ_LEN);
}

static int32_t aw8680x_sram_cycle_write(struct aw8680x *p_aw8680x,
						int cycle_num, int effect_len)
{
	uint32_t valid_data_addr =
			p_aw8680x->sram_bin->header_info[0].valid_data_addr;
	uint32_t download_addr =
			p_aw8680x->sram_bin->header_info[0].download_addr;
	uint32_t valid_addr_offset =
				valid_data_addr + cycle_num * WRITE_SRAM_MAX;
	uint32_t download_addr_offset =
				download_addr + WRITE_SRAM_MAX * cycle_num;

	memcpy(p_aw8680x->p_gui_data_s.soc_data,
		&(p_aw8680x->sram_bin->info.data[valid_addr_offset]),
		effect_len);
	aw8680x_set_common_info(p_aw8680x, effect_len, RAM_ID, RAM_WRITE_ID);

	return aw8680x_soc_protocol_set(p_aw8680x, download_addr_offset,
								SOC_READ_LEN);
}

static int32_t
aw8680x_flash_write_bin_to_soc(struct aw8680x *p_aw8680x, uint8_t flash_flag)
{
	int32_t i = DATA_INIT;
	int32_t ret = DATA_INIT;
	int32_t flash_write_count = DATA_INIT;
	uint16_t flash_write_last = DATA_INIT;
	uint32_t valid_data_len = DATA_INIT;

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE)
		valid_data_len =
		p_aw8680x->flash_boot_bin->header_info[0].valid_data_len;
	else if (flash_flag == AW8680X_FLASH_APP_UPDATE)
		valid_data_len =
			p_aw8680x->flash_app_bin->header_info[0].valid_data_len;

	if (valid_data_len > WRITE_FLASH_MAX) {
		flash_write_count = valid_data_len / WRITE_FLASH_MAX;
		flash_write_last = valid_data_len % WRITE_FLASH_MAX;
	} else {
		flash_write_count = DATA_INIT;
	}

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
		AWLOGI(p_aw8680x->dev, "flash boot bin flash write count = %d",
							flash_write_count);
		AWLOGI(p_aw8680x->dev, "flash boot bin flash write last = %d",
							flash_write_last);
	} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
		AWLOGI(p_aw8680x->dev, "flash app bin flash write count = %d",
							flash_write_count);
		AWLOGI(p_aw8680x->dev, "flash app bin flash write last = %d",
							flash_write_last);
	}

	for (i = DATA_INIT; i < flash_write_count; i++) {
		ret = aw8680x_flash_cycle_write(p_aw8680x, i, WRITE_FLASH_MAX,
								flash_flag);
		if (ret != AW_SUCCESS) {
			if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
				AWLOGE(p_aw8680x->dev, "flash boot bin cycle write fail!");
			} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
				AWLOGE(p_aw8680x->dev, "flash app bin cycle write fail!");
			}
			return ret;
		}
	}

	if (flash_write_last != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "flash last app write");
		ret = aw8680x_flash_cycle_write(p_aw8680x, flash_write_count,
						flash_write_last, flash_flag);
		if (ret != AW_SUCCESS) {
			if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
				AWLOGE(p_aw8680x->dev, "flash boot bin cycle write fail!");
			} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
				AWLOGE(p_aw8680x->dev, "flash app bin cycle write fail!");
			}
			return ret;
		}
		if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
			AWLOGI(p_aw8680x->dev, "flash boot bin cycle write Successfully!");
		} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
			AWLOGI(p_aw8680x->dev, "flash app bin cycle write Successfully!");
		}
	} else {
		if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
			AWLOGI(p_aw8680x->dev, "flash boot bin cycle write Successfully!");
		} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
			AWLOGI(p_aw8680x->dev, "flash app bin cycle write Successfully!");
		}
	}
	if (valid_data_len < WRITE_FLASH_MAX) {
		ret = aw8680x_flash_cycle_write(p_aw8680x, 0, valid_data_len,
								flash_flag);
		if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
			if (ret != AW_SUCCESS) {
				AWLOGE(p_aw8680x->dev, "flash boot bin cycle write fail!");
				return ret;
			}
			AWLOGI(p_aw8680x->dev, "flash boot bin cycle write Successfully!");
		} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
			if (ret != AW_SUCCESS) {
				AWLOGE(p_aw8680x->dev, "in flash app, flash cycle write fail!");
				return ret;
			}
			AWLOGI(p_aw8680x->dev, "flash app bin cycle write Successfully!");
		}
	}

	p_aw8680x->flash_update_ok = true;

	return AW_SUCCESS;

}

/*******************************************************************************
 * function : erase flash before updating new flash firmware,
		erase flash 512 bytes every time.
 * $flash_flag :
 * AW8680X_FLASH_BOOT_UPDATE : erase flash boot, flash boot space is 4k,
 * AW8680X_FLASH_BPP_UPDATE : erase flash app.
*******************************************************************************/
static int32_t
aw8680x_erase_flash_data(struct aw8680x *p_aw8680x, uint8_t flash_flag)
{
	int32_t ret = DATA_INIT;
	int32_t erase_num = DATA_INIT;
	int32_t erase_count = 3;
	uint32_t erase_base_addr = DATA_INIT;
	uint32_t valid_data_len = DATA_INIT;

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE) {
		erase_num = ERASE_FLASH_BOOT_SIZE;
		erase_base_addr = p_aw8680x->flash_boot_addr;
	} else if (flash_flag == AW8680X_FLASH_APP_UPDATE) {
		valid_data_len =
			p_aw8680x->flash_app_bin->header_info[0].valid_data_len;
		if (valid_data_len > ERASE_BYTE_MAX) {
			if ((valid_data_len % ERASE_BYTE_MAX) == DATA_INIT)
				erase_num = valid_data_len / ERASE_BYTE_MAX;
			else
				erase_num = valid_data_len / ERASE_BYTE_MAX + 1;
		} else {
			erase_num = 1;
		}
		erase_base_addr = p_aw8680x->flash_app_addr;
	}

	while (erase_count--) {
		AWLOGE(p_aw8680x->dev, "erase num = %d", erase_num);
		AWLOGE(p_aw8680x->dev, "erase_base_addr = %x", erase_base_addr);
		ret = aw8680x_erase_sector(p_aw8680x, erase_base_addr, erase_num);
		if (ret == AW_SUCCESS)
			break;
	}
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "erase flash app sector fail!");
		return ret;
	}

	if (flash_flag == AW8680X_FLASH_BOOT_UPDATE)
		AWLOGI(p_aw8680x->dev, "erase flash boot successfully!!!");
	else if (flash_flag == AW8680X_FLASH_APP_UPDATE)
		AWLOGI(p_aw8680x->dev, "erase flash app successfully!!!");

	return AW_SUCCESS;
}

static int32_t aw8680x_flash_app_bin_update_to_soc(struct aw8680x *p_aw8680x,
							uint8_t flash_flag)
{
	int32_t ret = DATA_INIT;
	int32_t update_count = 3;

	while (update_count--) {
		ret = aw8680x_erase_flash_data(p_aw8680x, flash_flag);
		if (ret != AW_SUCCESS)
			break;
		ret = aw8680x_flash_write_bin_to_soc(p_aw8680x, flash_flag);
		if (ret == AW_SUCCESS)
			break;
	}
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "update flash app fail");
		return ret;
	}
	AWLOGI(p_aw8680x->dev, "update flash app success");

	return AW_SUCCESS;
}

/*******************************************************************************
 * In flash app bin, the main information is:
 * $info.len : the size of bin
 * $info.data : all data of bin
 * $flash_boot_addr : base address of flash app which is saved in bin
 * $bin_data_type : the bin type
 * $header_info[0].app_version : the firmware version of flash app bin
 *****************************************************************************/
static int32_t aw8680x_flash_app_bin_parsed(struct aw8680x *p_aw8680x,
						const struct firmware *cont)
{
	int32_t ret = DATA_INIT;
	uint32_t bin_data_type = DATA_INIT;

	p_aw8680x->flash_app_bin = devm_kzalloc(p_aw8680x->dev,
				cont->size + sizeof(struct aw_bin), GFP_KERNEL);
	if (!(p_aw8680x->flash_app_bin)) {
		AWLOGE(p_aw8680x->dev, "AP failed allcate memory to flash app bin!");
		return -FLAH_APP_BIN_ERR;
	}
	p_aw8680x->flash_app_bin->info.len = cont->size;
	memcpy(p_aw8680x->flash_app_bin->info.data, cont->data, cont->size);
	release_firmware(cont);

	ret = aw_parsing_bin_file(p_aw8680x->flash_app_bin);
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "AP parse flash app bin failed!");
		return ret;
	}
	p_aw8680x->flash_app_addr = p_aw8680x->flash_app_bin->header_info[0].download_addr;
	bin_data_type = p_aw8680x->flash_app_bin->header_info[0].bin_data_type;
	AWLOGI(p_aw8680x->dev, "In flash app bin, down load base addr is : 0x%x", p_aw8680x->flash_app_addr);
	if (bin_data_type != SOC_APP_DATA_TYPE) {
		AWLOGE(p_aw8680x->dev, "soc app data type in prased flash app bin is error!");
		return -FLAH_APP_BIN_ERR;
	}
	if ((p_aw8680x->flash_app_addr < FLASH_APP_BASE_ADDR) || (p_aw8680x->flash_app_addr > FLASH_MAX_ADDR)) {
		AWLOGE(p_aw8680x->dev, "AP update flash app address err!!");
		AWLOGE(p_aw8680x->dev, "flash app download addr is 0x%x", p_aw8680x->flash_app_addr);
		return -FLAH_APP_BIN_ERR;
	}
	p_aw8680x->flash_app_version_in_bin = p_aw8680x->flash_app_bin->header_info[0].app_version;
	AWLOGI(p_aw8680x->dev, "In flash app bin, the app version is : V%x",
					p_aw8680x->flash_app_version_in_bin);
	AWLOGI(p_aw8680x->dev, "AP parse flash app bin success!");

	return AW_SUCCESS;
}

static void
aw8680x_flash_app_bin_loaded(const struct firmware *cont, void *context)
{
	int32_t ret = DATA_INIT;
	int32_t connect_count = 3;
	struct aw8680x *p_aw8680x = context;

	if (!cont) {
		AWLOGE(p_aw8680x->dev, "Can't find the bin file : %s!",
							aw8680x_flash_app_bin);
		release_firmware(cont);
		ret = aw8680x_jump_flash_app(p_aw8680x, FLASH_APP_BASE_ADDR);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, " jump flash fail");
			return;
		}
		AWLOGI(p_aw8680x->dev, "jump flash success");
		mdelay(FLASH_APP_VERSION_GET_TIME);
		while (connect_count--) {
			ret = aw8680x_connect(p_aw8680x);
			if (ret == AW_SUCCESS)
				break;
		}
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "after update connect fail!");
			return;
		}
		p_aw8680x->pc_point_flash_app = p_aw8680x->pc_location;
		AWLOGI(p_aw8680x->dev, "after update connect success :  pc location is 0x%x",
								p_aw8680x->pc_point_flash_app);
		enable_irq(gpio_to_irq(p_aw8680x->irq_gpio));
		return;
	}
	AWLOGI(p_aw8680x->dev, "Find the bin file : %s!", aw8680x_flash_app_bin);

	ret = aw8680x_flash_app_bin_parsed(p_aw8680x, cont);
	if (ret != AW_SUCCESS) {
		AWLOGI(p_aw8680x->dev, "flash app not need to update because flash app bin parsing failed!");
		release_firmware(cont);
		devm_kfree(p_aw8680x->dev, p_aw8680x->flash_app_bin);
		aw8680x_hw_reset(p_aw8680x);
		mdelay(FLASH_APP_INIT_TIME);
		return;
	}

	if (p_aw8680x->flash_app_version_get_flag == true) {
		if (p_aw8680x->flash_app_version_in_bin >
					p_aw8680x->flash_app_version_in_soc) {
			AWLOGI(p_aw8680x->dev, "flash app version in bin is higher flash app version in soc!");
			p_aw8680x->flash_app_update_flag = true;
		} else {
			p_aw8680x->flash_app_update_flag = false;
		}
	} else {
		p_aw8680x->flash_app_update_flag = true;
	}

	if (p_aw8680x->adb_update_flash == ADB_UPDATE_FLASH_APP)
		p_aw8680x->flash_app_update_flag = true;

	if (p_aw8680x->flash_app_update_flag == true) {
		AWLOGI(p_aw8680x->dev, "flash app need to update!!!");
		aw8680x_jump_sram(p_aw8680x, p_aw8680x->sram_addr);
		AWLOGI(p_aw8680x->dev, "sram addr: 0x%x", p_aw8680x->sram_addr);
		mdelay(SRAM_INIT_TIME);
		aw8680x_connect(p_aw8680x);
		aw8680x_flash_app_bin_update_to_soc(p_aw8680x,
						AW8680X_FLASH_APP_UPDATE);
	} else {
		AWLOGI(p_aw8680x->dev, "flash app not need to update!");
	}
	devm_kfree(p_aw8680x->dev, p_aw8680x->flash_app_bin);
	ret = aw8680x_jump_flash_app(p_aw8680x, FLASH_APP_BASE_ADDR);
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, " jump flash fail");
		return;
	}
	mdelay(FLASH_APP_INIT_TIME);
	AWLOGI(p_aw8680x->dev, "jump flash success");
	while (connect_count--) {
		ret = aw8680x_connect(p_aw8680x);
		if (ret == AW_SUCCESS)
			break;
	}
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "after update connect fail!");
		return;
	}
	p_aw8680x->pc_point_flash_app = p_aw8680x->pc_location;
	AWLOGI(p_aw8680x->dev, "after update connect success :  pc location is 0x%x",
							p_aw8680x->pc_point_flash_app);
	if (p_aw8680x->input_apply_flag == true)
		enable_irq(gpio_to_irq(p_aw8680x->irq_gpio));

}

static int32_t aw8680x_flash_app_bin_update(struct aw8680x *p_aw8680x)
{
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			aw8680x_flash_app_bin, p_aw8680x->dev, GFP_KERNEL,
			p_aw8680x, aw8680x_flash_app_bin_loaded);
}

static int32_t aw8680x_flash_boot_bin_update_to_soc(struct aw8680x *p_aw8680x,
							uint8_t flash_flag)
{
	int32_t ret = DATA_INIT;
	int32_t update_count = 3;

	while (update_count--) {
		ret = aw8680x_erase_flash_data(p_aw8680x, flash_flag);
		if (ret != AW_SUCCESS)
			break;
		ret = aw8680x_flash_write_bin_to_soc(p_aw8680x, flash_flag);
		if (ret == AW_SUCCESS)
			break;
	}
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "update flash boot fail");
		return ret;
	}
	AWLOGI(p_aw8680x->dev, "update flash boot success");

	return ret;
}

/*******************************************************************************
 * In flash boot bin, the main information is:
 * $info.len : the size of bin
 * $info.data : all data of bin
 * $flash_boot_addr : base address of flash boot which is saved in bin
 * $bin_data_type : the bin type
 * $header_info[0].app_version : the firmware version of flash boot bin
 *****************************************************************************/
static int32_t aw8680x_flash_boot_bin_parsed(struct aw8680x *p_aw8680x,
						const struct firmware *cont)
{
	int32_t ret = DATA_INIT;
	uint32_t bin_data_type = DATA_INIT;

	p_aw8680x->flash_boot_bin = devm_kzalloc(p_aw8680x->dev,
				cont->size + sizeof(struct aw_bin), GFP_KERNEL);
	if (!(p_aw8680x->flash_boot_bin)) {
		AWLOGE(p_aw8680x->dev, "failed to allcating memory!");
		return -FLAH_BOOT_BIN_ERR;
	}
	p_aw8680x->flash_boot_bin->info.len = cont->size;
	memcpy(p_aw8680x->flash_boot_bin->info.data, cont->data, cont->size);

	ret = aw_parsing_bin_file(p_aw8680x->flash_boot_bin);
	if (ret != DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "AP parse flash boot bin failed!!");
		return -FLAH_BOOT_BIN_ERR;
	}

	p_aw8680x->flash_boot_addr =
			p_aw8680x->flash_boot_bin->header_info[0].download_addr;
	bin_data_type = p_aw8680x->flash_boot_bin->header_info[0].bin_data_type;
	AWLOGI(p_aw8680x->dev, "In flash boot bin, flash boot base addr = 0x%x",
						p_aw8680x->flash_boot_addr);
	if (bin_data_type != SOC_APP_DATA_TYPE) {
		AWLOGE(p_aw8680x->dev, "bin not soc protcol!");
		return -FLAH_BOOT_BIN_ERR;
	}
	if ((p_aw8680x->flash_boot_addr < FLASH_BOOT_BASE_ADDR) ||
			(p_aw8680x->flash_boot_addr > FLASH_APP_BASE_ADDR)) {
		AWLOGE(p_aw8680x->dev, "flash boot bin address err");
		AWLOGE(p_aw8680x->dev, "flash boot bin downloadaddr is 0x%x",
						p_aw8680x->flash_boot_addr);
		return -FLAH_BOOT_BIN_ERR;
	}
	p_aw8680x->flash_boot_version_in_bin =
			p_aw8680x->flash_boot_bin->header_info[0].app_version;
	AWLOGI(p_aw8680x->dev, "the version of flash boot in bin is : V%x",
					p_aw8680x->flash_boot_version_in_bin);
	AWLOGI(p_aw8680x->dev, "AP parse flash boot bin success!");

	return AW_SUCCESS;
}

static void aw8680x_flash_bin_loaded(const struct firmware *cont, void *context)
{
	int32_t ret = DATA_INIT;
	int32_t connect_count = 3;
	struct aw8680x *p_aw8680x = context;

	if (!cont) {
		AWLOGE(p_aw8680x->dev, "Can't find the bin file : %s!",
							aw8680x_flash_boot_bin);
		release_firmware(cont);
		aw8680x_flash_app_bin_update(p_aw8680x);
		return;
	}
	AWLOGI(p_aw8680x->dev, "Find the bin file : %s!",
							aw8680x_flash_boot_bin);

	ret = aw8680x_flash_boot_bin_parsed(p_aw8680x, cont);
	release_firmware(cont);
	if (ret != AW_SUCCESS) {
		aw8680x_flash_app_bin_update(p_aw8680x);
		return;
	}

	if (p_aw8680x->adb_update_flash == ADB_UPDATE_FLASH_BOOT) {
		p_aw8680x->flash_boot_update_flag = true;
		AWLOGI(p_aw8680x->dev, "flash boot need to update!");
	} else {
		if (p_aw8680x->flash_boot_version_get_flag == true) {
			if (p_aw8680x->flash_boot_version_in_bin >
					p_aw8680x->flash_boot_version_in_soc) {
				AWLOGI(p_aw8680x->dev, "flash boot need to update!");
				p_aw8680x->flash_boot_update_flag = true;
			} else {
				AWLOGI(p_aw8680x->dev, "flash boot not need to update!");
				p_aw8680x->flash_boot_update_flag = false;
			}
		} else {
			p_aw8680x->flash_boot_update_flag = false;
			AWLOGI(p_aw8680x->dev, "flash boot not need to update!");
		}
	}

	if (p_aw8680x->flash_boot_update_flag == true)
		aw8680x_flash_boot_bin_update_to_soc(p_aw8680x,
						AW8680X_FLASH_BOOT_UPDATE);

	devm_kfree(p_aw8680x->dev, p_aw8680x->flash_boot_bin);

	if (p_aw8680x->adb_update_flash != ADB_UPDATE_FLASH_BOOT) {
		aw8680x_flash_app_bin_update(p_aw8680x);
	} else {
		ret = aw8680x_jump_flash_app(p_aw8680x, FLASH_APP_BASE_ADDR);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, " jump flash fail");
			return;
		}
		mdelay(FLASH_APP_INIT_TIME);
		AWLOGI(p_aw8680x->dev, "jump flash success");
		while (connect_count--) {
			ret = aw8680x_connect(p_aw8680x);
			if (ret == AW_SUCCESS)
				break;
		}
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "after update connect fail!");
			return;
		}
		p_aw8680x->pc_point_flash_app = p_aw8680x->pc_location;
		AWLOGI(p_aw8680x->dev, "after update connect success :  pc location is 0x%x",
								p_aw8680x->pc_point_flash_app);
		if (p_aw8680x->input_apply_flag == true)
			enable_irq(gpio_to_irq(p_aw8680x->irq_gpio));
	}
}

static int32_t aw8680x_flash_bin_update(struct aw8680x *p_aw8680x)
{
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			aw8680x_flash_boot_bin, p_aw8680x->dev, GFP_KERNEL,
			p_aw8680x, aw8680x_flash_bin_loaded);

}

/*******************************************************************************
 * function : if update failed firstly, retry update sram until three times.
 ******************************************************************************/
static int32_t aw8680x_sram_write_bin_to_soc(struct aw8680x *p_aw8680x)
{
	uint16_t write_last = DATA_INIT;
	int32_t i = DATA_INIT;
	int32_t ret = DATA_INIT;
	int32_t write_count = DATA_INIT;
	uint32_t valid_data_len = DATA_INIT;

	valid_data_len = p_aw8680x->sram_bin->header_info[0].valid_data_len;
	AWLOGI(p_aw8680x->dev, "the valid size of sram bin is %d",
								valid_data_len);
	if (valid_data_len > WRITE_SRAM_MAX) {
		write_count = valid_data_len / WRITE_SRAM_MAX;
		write_last = valid_data_len % WRITE_SRAM_MAX;
		AWLOGI(p_aw8680x->dev, "sram write count is %d", write_count);
		AWLOGI(p_aw8680x->dev, "sram write last is %d", write_last);
		for (i = DATA_INIT; i < write_count; i++) {
			ret = aw8680x_sram_cycle_write(p_aw8680x, i,
								WRITE_SRAM_MAX);
			if (ret != AW_SUCCESS) {
				AWLOGE(p_aw8680x->dev, "sram cycle write fail");
				return ret;
			}
		}
		if (write_last != DATA_INIT) {
			ret = aw8680x_sram_cycle_write(p_aw8680x, write_count,
								write_last);
			if (ret != AW_SUCCESS) {
				AWLOGE(p_aw8680x->dev, "sram cycle write fail");
				return ret;
			}
		}
	} else {
		ret = aw8680x_sram_cycle_write(p_aw8680x, 0, valid_data_len);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "sram cycle write fail!");
			return ret;
		}
	}
	AWLOGI(p_aw8680x->dev, "Successfully write all data to sram!!!");

	return AW_SUCCESS;
}

/*******************************************************************************
 * function : after parsing sram bin, update to sram in soc.
 * before updating sram need to jump flash boot.
 ******************************************************************************/
static int32_t aw8680x_sram_bin_retry_update(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t sram_update_retry_cont = 3;

	while (sram_update_retry_cont != DATA_INIT) {
		ret = aw8680x_sram_write_bin_to_soc(p_aw8680x);
		if (ret == AW_SUCCESS)
			return AW_SUCCESS;
		sram_update_retry_cont--;
		udelay(5);
	}
	AWLOGE(p_aw8680x->dev, "sram update error!");

	return ret;
}

static int32_t aw8680x_sram_bin_update_to_soc(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	ret = aw8680x_jump_boot(p_aw8680x);
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "soc jump boot failed!");
		return -JUMP_BOOT_FAILED;
	}
	AWLOGI(p_aw8680x->dev, "soc jump boot successfully!!");

	if ((p_aw8680x->pc_location != PC_POINT_ROM_BOOT) &&
			(p_aw8680x->pc_location != PC_POINT_FLASH_BOOT)) {
		AWLOGE(p_aw8680x->dev, "soc jump boot failed!, pc current location is 0x%x",
							p_aw8680x->pc_location);
			return -JUMP_BOOT_FAILED;
	}

	ret = aw8680x_sram_bin_retry_update(p_aw8680x);
	if (ret != AW_SUCCESS)
		return -SRAM_BIN_FAILED;

	return AW_SUCCESS;
}

static int32_t
aw8680x_sram_bin_parsed(struct aw8680x *p_aw8680x, const struct firmware *cont)
{
	int32_t ret = DATA_INIT;

	p_aw8680x->sram_bin = devm_kzalloc(p_aw8680x->dev, cont->size +
					sizeof(struct aw_bin), GFP_KERNEL);
	if (!(p_aw8680x->sram_bin)) {
		AWLOGE(p_aw8680x->dev, "failed to allcating memory!");
		return -SRAM_BIN_FAILED;
	}
	p_aw8680x->sram_bin->info.len = cont->size;
	memcpy(p_aw8680x->sram_bin->info.data, cont->data, cont->size);
	release_firmware(cont);
	ret = aw_parsing_bin_file(p_aw8680x->sram_bin);
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "parse sram bin fail!");
		return -SRAM_BIN_FAILED;
	}
	p_aw8680x->sram_addr = p_aw8680x->sram_bin->header_info[0].download_addr;
	if (p_aw8680x->sram_bin->header_info[0].bin_data_type !=
							SOC_APP_DATA_TYPE) {
			AWLOGE(p_aw8680x->dev, "SOC APP DATA TYPE error!");
			return -SRAM_BIN_FAILED;
	}
	if ((p_aw8680x->sram_addr < SRAM_BASE_ADDR) ||
				(p_aw8680x->sram_addr > SRAM_MAX_ADDR)) {
		AWLOGE(p_aw8680x->dev, "update sram address err!");
		AWLOGE(p_aw8680x->dev, "sram download_addr is 0x%x",
							p_aw8680x->sram_addr);
		return -SRAM_BIN_FAILED;
	}
	AWLOGE(p_aw8680x->dev, "parse sram bin successfully!");

	return AW_SUCCESS;
}

static int32_t
aw8680x_srm_bin_loaded(struct aw8680x *p_aw8680x, const struct firmware *cont)
{
	int32_t ret = DATA_INIT;

	ret = aw8680x_sram_bin_parsed(p_aw8680x, cont);
	if (ret != AW_SUCCESS)
		return ret;

	ret = aw8680x_sram_bin_update_to_soc(p_aw8680x);
	if (ret != AW_SUCCESS)
		return ret;

	return AW_SUCCESS;
}

/*******************************************************************************
 * the function is used to load sram bin and load flash bin from ap to soc.
 ******************************************************************************/
static void aw8680x_cfg_loaded(const struct firmware *cont, void *context)
{
	int32_t ret = DATA_INIT;
	struct aw8680x *p_aw8680x = context;

	AWLOGI(p_aw8680x->dev, "AP start to get sram bin from system");

	if (!cont) {
		AWLOGE(p_aw8680x->dev, "Can't find the bin file : %s!",
							aw8680x_sram_bin);
		release_firmware(cont);
		aw8680x_connect(p_aw8680x);
		p_aw8680x->pc_point_flash_app = p_aw8680x->pc_location;
		AWLOGI(p_aw8680x->dev, "pc location is 0x%x", p_aw8680x->pc_point_flash_app);
		return;
	}
	AWLOGI(p_aw8680x->dev, "Find the bin file : %s!!", aw8680x_sram_bin);
	ret = aw8680x_srm_bin_loaded(p_aw8680x, cont);
	devm_kfree(p_aw8680x->dev, p_aw8680x->sram_bin);
	if (ret != AW_SUCCESS) {
		aw8680x_connect(p_aw8680x);
		p_aw8680x->pc_point_flash_app = p_aw8680x->pc_location;
		AWLOGI(p_aw8680x->dev, "pc location is 0x%x", p_aw8680x->pc_point_flash_app);
		return;
	}

	ret = aw8680x_jump_sram(p_aw8680x, p_aw8680x->sram_addr);
	if (ret == AW_SUCCESS) {
		aw8680x_connect(p_aw8680x);
		if (p_aw8680x->pc_location == PC_POINT_SRAM) {
			AWLOGI(p_aw8680x->dev, "jump sram ok!!");
		} else {
			AWLOGE(p_aw8680x->dev, "jump sram failed, pc point %x",
							p_aw8680x->pc_location);
			aw8680x_hw_reset(p_aw8680x);
			return;
		}
	} else {
		aw8680x_hw_reset(p_aw8680x);
		return;
	}
	mdelay(SRAM_INIT_TIME);

	if ((p_aw8680x->flash_boot_func == false) ||
			(p_aw8680x->adb_update_flash == ADB_UPDATE_FLASH_APP)) {
		aw8680x_flash_app_bin_update(p_aw8680x);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "request flash app bin firmware failed!");
			aw8680x_hw_reset(p_aw8680x);
		}
		return;
	}

	ret = aw8680x_flash_bin_update(p_aw8680x);
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "request flash app bin firmware failed!");
		aw8680x_flash_app_bin_update(p_aw8680x);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "request flash app bin firmware failed!");
			aw8680x_hw_reset(p_aw8680x);
		}
	}

}

static int32_t aw8680x_bin_update(struct aw8680x *p_aw8680x)
{
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			aw8680x_sram_bin, p_aw8680x->dev, GFP_KERNEL,
			p_aw8680x, aw8680x_cfg_loaded);
}

/*******************************************************************************
 * use bin from system docunment which must be completely.
 * the reason is why need to use 5 seconds delay work.
 ******************************************************************************/
static void aw8680x_bin_work_routine(struct work_struct *work)
{
	ssize_t ret_fir = DATA_INIT;
	int32_t ret = DATA_INIT;
	struct aw8680x *p_aw8680x =
			container_of(work, struct aw8680x, bin_work.work);

	AWLOGI(p_aw8680x->dev, "enter");

	ret = aw8680x_connect(p_aw8680x);
	if (ret != AW_SUCCESS) {
		ret = aw8680x_jump_boot(p_aw8680x);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "connect failed! ret = %d", ret);
			return;
		}
	}
	AWLOGI(p_aw8680x->dev, "pc location is : %x", p_aw8680x->pc_location);

	if ((p_aw8680x->pc_location == PC_POINT_FLASH_BOOT) ||
			(p_aw8680x->pc_location == PC_POINT_ROM_BOOT)) {
		ret = aw8680x_jump_flash_app(p_aw8680x, FLASH_APP_BASE_ADDR);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "jump flash err!");
			p_aw8680x->jump_flash_app_flag = false;
		} else {
			AWLOGI(p_aw8680x->dev, "jump flash OK!!");
			p_aw8680x->jump_flash_app_flag = true;
			mdelay(FLASH_APP_VERSION_GET_TIME);
		}
	} else {
		AWLOGI(p_aw8680x->dev, "pc point flash app");
		p_aw8680x->jump_flash_app_flag = true;
	}

	if (p_aw8680x->jump_flash_app_flag == true) {
		ret = aw8680x_flash_app_version_in_soc_get(p_aw8680x);
		if (ret != AW_SUCCESS)
			p_aw8680x->flash_app_version_get_flag = false;
		else
			p_aw8680x->flash_app_version_get_flag = true;
		AWLOGI(p_aw8680x->dev, "flash app version in soc is : V%x",
					p_aw8680x->flash_app_version_in_soc);
	}

	ret_fir = aw8680x_bin_update(p_aw8680x);
	if (ret_fir != AW_SUCCESS)
		AWLOGE(p_aw8680x->dev, "request sram bin firmware failed!");
}

static void aw8680x_bin_init(struct aw8680x *p_aw8680x, int32_t cfg_timer_val)
{
	INIT_DELAYED_WORK(&p_aw8680x->bin_work, aw8680x_bin_work_routine);
	schedule_delayed_work(&p_aw8680x->bin_work,
					msecs_to_jiffies(cfg_timer_val));
}

/******************************************************
 *
 * attribute : Used when debugging adb
 *
 ******************************************************/
static ssize_t aw8680x_connect_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int connect_count = 5;
	int ret = -1;

	aw8680x_wake_pin(p_aw8680x);
	while (connect_count--) {
		ret = aw8680x_connect(p_aw8680x);
		if (ret == 0)
			break;
	}
	if (ret == 0) {
		len += snprintf(buf + len, PAGE_SIZE - len,
							"connect success!\n");
		len += snprintf(buf + len, PAGE_SIZE - len,
					"pc location is 0x%x\n",
					p_aw8680x->pc_location);
		if ((p_aw8680x->pc_location == PC_POINT_ROM_BOOT)
			|| (p_aw8680x->pc_location == PC_POINT_FLASH_BOOT)) {
			len += snprintf(buf + len, PAGE_SIZE - len,
							"pc point boot!\n");
		} else if (p_aw8680x->pc_location ==
						p_aw8680x->pc_point_flash_app) {
			len += snprintf(buf + len, PAGE_SIZE - len,
							"pc point flash app!\n");
		} else {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"pc point not uboot or flash!!!\n");
		}
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "connect fail!\n");
	}

	return len;
}

static ssize_t aw8680x_update_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (p_aw8680x->input_apply_flag == true)
		disable_irq(gpio_to_irq(p_aw8680x->irq_gpio));

	if (val == ADB_UPDATE_FLASH_BOOT) {
		if (p_aw8680x->flash_boot_func == false) {
			AWLOGI(p_aw8680x->dev, "flash boot update function is unsupport!");
			return count;
		} else {
			if (p_aw8680x->flash_update_ok == false) {
				AWLOGE(p_aw8680x->dev, "update flash is not ok, please wait!");
				return count;
			} else {
				AWLOGI(p_aw8680x->dev, "update flash boot");
			}
		}
	} else if (val == ADB_UPDATE_FLASH_APP) {
		if (p_aw8680x->flash_update_ok == false) {
			AWLOGE(p_aw8680x->dev, "update flash is not ok, please wait!");
			return count;
		} else {
			AWLOGI(p_aw8680x->dev, "update flash app");
		}
	} else {
		AWLOGE(p_aw8680x->dev, "adb information err!");
		return count;
	}
	p_aw8680x->adb_update_flash = val;
	p_aw8680x->flash_update_ok = false;
	aw8680x_bin_init(p_aw8680x, AW8680X_ADB_BIN_INIT_DELAY);

	return count;
}

static ssize_t aw8680x_jump_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	int32_t jump_count = 3;
	int32_t ret = -1;
	int8_t databuf[10] = { 0 };

	AWLOGI(p_aw8680x->dev, "pc location is %x, pc_point_flash_app = %x",
			p_aw8680x->pc_location, p_aw8680x->pc_point_flash_app);

	if (p_aw8680x->input_apply_flag == true)
		disable_irq(gpio_to_irq(p_aw8680x->irq_gpio));

	if (sscanf(buf, "%s", databuf) == 1) {
		if ((strcmp(databuf, "flash_app") == AW_SUCCESS)
		&& (p_aw8680x->pc_location != p_aw8680x->pc_point_flash_app)) {
			AWLOGI(p_aw8680x->dev, "pc point not flash app");
			while (jump_count--) {
				mdelay(FLASH_BOOT_INIT_TIME);
				ret = aw8680x_jump_flash_app(p_aw8680x,
							FLASH_APP_BASE_ADDR);
				if (ret == AW_SUCCESS) {
					AWLOGI(p_aw8680x->dev, "jump flash app OK!!");
					mdelay(FLASH_APP_INIT_TIME);
					break;
				}
			}
		} else if ((strcmp(databuf, "boot") == AW_SUCCESS) &&
			(p_aw8680x->pc_location != PC_POINT_ROM_BOOT) &&
			(p_aw8680x->pc_location != PC_POINT_FLASH_BOOT)) {
			AWLOGI(p_aw8680x->dev, "pc point not flash boot");
			aw8680x_hw_reset(p_aw8680x);
			aw8680x_stay_boot(p_aw8680x);
		} else {
			AWLOGE(p_aw8680x->dev, "Do not known jump where");
		}
	}
	if (p_aw8680x->input_apply_flag == true)
		enable_irq(gpio_to_irq(p_aw8680x->irq_gpio));

	return count;
}

static ssize_t aw8680x_reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 1) {
		AWLOGI(p_aw8680x->dev, "enter adb reset");
		aw8680x_hw_reset(p_aw8680x);
		aw8680x_stay_boot(p_aw8680x);
	}

	return count;
}

static ssize_t aw8680x_wake_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 1) {
		AWLOGI(p_aw8680x->dev, "enter adb wake");
		aw8680x_wake_pin(p_aw8680x);
	}

	return count;
}

static ssize_t aw8680x_scan_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char databuf[10] = { 0 };
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);

	if (sscanf(buf, "%s", databuf) == 1) {
		if (strcmp(databuf, "active") == 0) {
			AWLOGI(p_aw8680x->dev, "enter active mode");
			aw8680x_active_mode_set(p_aw8680x);
		} else if (strcmp(databuf, "idle") == 0) {
			AWLOGI(p_aw8680x->dev, "enter idle mode");
			aw8680x_idle_mode_set(p_aw8680x);
		} else if (strcmp(databuf, "sleep") == 0) {
			AWLOGI(p_aw8680x->dev, "enter sleep mode");
			aw8680x_sleep_mode_set(p_aw8680x);
		} else {
			AWLOGE(p_aw8680x->dev, "unsupported mode!");
		}
	}

	return count;
}

static ssize_t aw8680x_adc_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_adc_data(p_aw8680x);
	if (ret != 0) {
		AWLOGE(p_aw8680x->dev, "adc data read fail!");
	} else {
		for (i = 0; i < ADC_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"adc data[%d] = 0x%x\n",
					i, p_aw8680x->info.adc_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_raw_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_raw_data(p_aw8680x);
	if (ret != 0) {
		AWLOGE(p_aw8680x->dev, "raw data read fail!");
	} else {
		for (i = 0; i < RAW_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"raw data[%d] = 0x%x\n",
					i, p_aw8680x->info.raw_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_force_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_force_data(p_aw8680x);
	if (ret != 0) {
		AWLOGE(p_aw8680x->dev, "force data read fail!");
	} else {
		for (i = 0; i < FORCE_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"force data[%d] = 0x%x\n",
					i, p_aw8680x->info.force_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_base_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_base_data(p_aw8680x);
	if (ret != 0) {
		AWLOGE(p_aw8680x->dev, "base data read fail!");
	} else {
		for (i = 0; i < BASE_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"base data[%d] = 0x%x\n",
					i, p_aw8680x->info.base_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_diff_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_diff_data(p_aw8680x);
	if (ret != 0) {
		AWLOGE(p_aw8680x->dev, "diff data read fail!");
	} else {
		for (i = 0; i < DIFF_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"diff_data[%d] = 0x%x",
					i, p_aw8680x->info.diff_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_sensor_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw8680x *p_aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int read_len = 0;
	uint8_t reg_val = 0;
	int i = 0;
	int read_count = 5;

	reg_val = 0xa4;
	aw8680x_register_i2c_writes(p_aw8680x, 0xf6, &reg_val, 1);
	reg_val = 0x0;
	aw8680x_register_i2c_writes(p_aw8680x, 0xf7, &reg_val, 1);
	while (read_count--) {
		aw8680x_register_i2c_reads(p_aw8680x, 0xf7, 1);
		if (p_aw8680x->read_data[0] != 0)
			break;
		udelay(10);
	}
	read_len = p_aw8680x->read_data[0];
	len += snprintf(buf + len, PAGE_SIZE - len, "read len = %d",
								read_len);
	aw8680x_register_i2c_reads(p_aw8680x, 0xf8, read_len);
	for (i = 0; i < read_len; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "the data[%d] = %d",
					i, p_aw8680x->read_data[i]);
	}
	return len;
}


static DEVICE_ATTR(connect, S_IWUSR | S_IRUGO, aw8680x_connect_show, NULL);
static DEVICE_ATTR(update, S_IWUSR | S_IRUGO, NULL, aw8680x_update_store);
static DEVICE_ATTR(jump, S_IWUSR | S_IRUGO, NULL, aw8680x_jump_store);
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, NULL, aw8680x_reset_store);
static DEVICE_ATTR(wake, S_IWUSR | S_IRUGO, NULL, aw8680x_wake_store);
static DEVICE_ATTR(scan_mode, S_IWUSR | S_IRUGO, NULL, aw8680x_scan_mode_store);
static DEVICE_ATTR(adc_data, S_IWUSR | S_IRUGO, aw8680x_adc_data_show, NULL);
static DEVICE_ATTR(raw_data, S_IWUSR | S_IRUGO, aw8680x_raw_data_show, NULL);
static DEVICE_ATTR(force_data, S_IWUSR | S_IRUGO, aw8680x_force_data_show,
									NULL);
static DEVICE_ATTR(base_data, S_IWUSR | S_IRUGO, aw8680x_base_data_show, NULL);
static DEVICE_ATTR(diff_data, S_IWUSR | S_IRUGO, aw8680x_diff_data_show, NULL);
static DEVICE_ATTR(sensor_status, S_IWUSR | S_IRUGO,
				aw8680x_sensor_status_show, NULL);

static struct attribute *aw8680x_attributes[] = {
	&dev_attr_connect.attr,
	&dev_attr_update.attr,
	&dev_attr_jump.attr,
	&dev_attr_reset.attr,
	&dev_attr_wake.attr,
	&dev_attr_scan_mode.attr,
	&dev_attr_adc_data.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_force_data.attr,
	&dev_attr_base_data.attr,
	&dev_attr_diff_data.attr,
	&dev_attr_sensor_status.attr,
	NULL
};

static struct attribute_group aw8680x_attribute_group = {
	.attrs = aw8680x_attributes
};

/*****************************************************
 *
 * aw8680x apk interface
 *
 *****************************************************/
static int32_t aw8680x_file_open(struct inode *inode, struct file *filp)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	filp->private_data = (void *)g_aw8680x;

	return 0;
}

static int32_t aw8680x_file_release(struct inode *inode, struct file *filp)
{
	filp->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static ssize_t aw8680x_file_read(struct file *filp, char *buff, size_t len,
				 loff_t *offset)
{
	struct aw8680x *p_aw8680x = (struct aw8680x *)filp->private_data;
	int32_t i = DATA_INIT;
	int32_t ret = DATA_INIT;
	uint8_t *pbuff = NULL;
	uint8_t reg_addr = DATA_INIT;

	AWLOGI(p_aw8680x->dev, "enter");

	if (len > 256)
		return len;
	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		AWLOGE(p_aw8680x->dev,  "alloc memory fail");
		return len;
	}
	/* get reg addr */
	if (copy_from_user(&reg_addr, buff, 1)) {
		kfree(pbuff);
		return len;
	}
	AWLOGI(p_aw8680x->dev, "reg_addr is 0x%x", reg_addr);
	AWLOGI(p_aw8680x->dev, "read_len is %zu", len);
	ret = aw8680x_register_i2c_reads(p_aw8680x, reg_addr, len);
	if (ret < DATA_INIT) {
		AWLOGE(p_aw8680x->dev, "failed to read data, ret is : %d", ret);
		kfree(pbuff);
		return len;
	}
	for (i = DATA_INIT; i < len; i++) {
		pbuff[i] = p_aw8680x->read_data[i];
		AWLOGI(p_aw8680x->dev, "pbuff[%d] = 0x%02x", i, pbuff[i]);
	}
	ret = copy_to_user(buff + 1, pbuff, len);
	if (ret) {
		AWLOGI(p_aw8680x->dev, "copy to user fail");
		kfree(pbuff);
		return len;
	}

	kfree(pbuff);
	return len;
}

static ssize_t aw8680x_file_write(struct file *filp, const char *buff,
				  size_t len, loff_t *off)
{
	struct aw8680x *p_aw8680x = (struct aw8680x *)filp->private_data;
	int32_t i = DATA_INIT;
	int32_t ret = DATA_INIT;
	unsigned char *pbuff = NULL;
	unsigned char reg_addr = DATA_INIT;

	AWLOGI(p_aw8680x->dev, "enter");

	if (len > 256)
		return len;
	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		AWLOGE(p_aw8680x->dev, "alloc memory fail");
		return len;
	}
	/* get reg addr */
	ret = copy_from_user(&reg_addr, buff, 1);
	if (ret) {
		AWLOGE(p_aw8680x->dev, "copy from user reg_addr fail");
		kfree(pbuff);
		return len;
	}
	AWLOGI(p_aw8680x->dev, "reg_addr is 0x%x", reg_addr);
	AWLOGI(p_aw8680x->dev, "write_len is %zu", len);
	/* get reg data */
	ret = copy_from_user(pbuff, buff + 1, len);
	if (ret) {
		AWLOGE(p_aw8680x->dev, "copy from user reg_data fail");
		kfree(pbuff);
		return len;
	}
	for (i = 0; i < len; i++)
		AWLOGI(p_aw8680x->dev, "pbuff[%d] = 0x%02x", i, pbuff[i]);
	ret = aw8680x_register_i2c_writes(p_aw8680x, reg_addr, pbuff, len);
	if (ret < 0) {
		AWLOGE(p_aw8680x->dev, "failed to write data, ret is : %d",
									ret);
		kfree(pbuff);
		return len;
	}

	kfree(pbuff);
	return len;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw8680x_file_read,
	.write = aw8680x_file_write,
	.open = aw8680x_file_open,
	.release = aw8680x_file_release,
};

static struct miscdevice aw8680x_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW8680X_I2C_NAME,
	.fops = &fops,
};

static int32_t aw8680x_file_init(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	ret = misc_register(&aw8680x_misc);
	if (ret) {
		AWLOGE(p_aw8680x->dev, "misc fail, ret is : %d", ret);
		return ret;
	}
	return AW_SUCCESS;
}

/*****************************************************
 *
 * aw8680x irq
 *
 *****************************************************/
static irqreturn_t aw8680x_irq(int irq, void *data)
{
	struct aw8680x *p_aw8680x = data;
	int32_t ret = DATA_INIT;

	AWLOGI(p_aw8680x->dev, "pc location is 0x%x, pc point flash app is : 0x%x",
			p_aw8680x->pc_location, p_aw8680x->pc_point_flash_app);
	if (p_aw8680x->pc_location == p_aw8680x->pc_point_flash_app) {
		ret = aw8680x_get_key_data(p_aw8680x);
		if (ret == AW_SUCCESS) {
			AWLOGI(p_aw8680x->dev, "key data : 0x%x",
						p_aw8680x->info.key_data);
			if (p_aw8680x->info.key_data == 0x01) {
				input_report_key(p_aw8680x->input, KEY_UP, 1);
				input_sync(p_aw8680x->input);
			}
		} else {
			AWLOGE(p_aw8680x->dev, "key data read fail!");
		}
	} else {
		AWLOGE(p_aw8680x->dev, "pc point not in flash app!");
	}

	return IRQ_HANDLED;
}

static int32_t aw8680x_read_chipid_once(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	aw8680x_set_common_info(p_aw8680x, SOC_DATA_LEN, HANDSHAKE_ID, CHIP_ID);
	ret = aw8680x_soc_protocol_set(p_aw8680x, SOC_ADDR, SOC_READ_LEN);
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "Get chipid failed!!! ret is : %d", ret);
		return ret;
	}

	if (p_aw8680x->chipid_data == AW86801) {
		AWLOGI(p_aw8680x->dev, "aw8680x detected, the soc is AW86801!");
		return AW_SUCCESS;
	} else if (p_aw8680x->chipid_data == AW86802) {
		AWLOGI(p_aw8680x->dev, "aw8680x detected, the soc is AW86802!");
		return AW_SUCCESS;
	} else if (p_aw8680x->chipid_data == AW86802A) {
		AWLOGI(p_aw8680x->dev, "aw8680x detected,the soc is AW86802A!");
		return AW_SUCCESS;
	} else if (p_aw8680x->chipid_data == AW86803) {
		AWLOGI(p_aw8680x->dev, "aw8680x detected, the soc is AW86803!");
		return AW_SUCCESS;
	} else {
		AWLOGE(p_aw8680x->dev, "unsupport dev, chipid is (0x%06x)",
							p_aw8680x->chipid_data);
	}

	return -CHIPID_ERR;
}

static int32_t aw8680x_read_chipid(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;
	int32_t read_chipid_time_retry = DATA_INIT;

	while (read_chipid_time_retry < READ_CHIPID_RETRY_TIME) {
		aw8680x_hw_reset(p_aw8680x);
		aw8680x_stay_boot(p_aw8680x);
		ret = aw8680x_read_chipid_once(p_aw8680x);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "read chipid retry time : %d",
							read_chipid_time_retry);
			read_chipid_time_retry++;
		} else {
			return AW_SUCCESS;
		}
	}

	return -CHIPID_ERR;
}


/*****************************************************
 *
 * device tree
 *
 *****************************************************/
/* Get the irq port number in the device tree */
static int32_t
aw8680x_reset_gpio_set(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;

	p_aw8680x->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(p_aw8680x->reset_gpio)) {
		AWLOGI(p_aw8680x->dev, "reset gpio provided ok!!");
		ret = devm_gpio_request_one(p_aw8680x->dev,
					p_aw8680x->reset_gpio,
					GPIOF_OUT_INIT_LOW,
					"aw8680x_rst");
		if (ret) {
			AWLOGE(p_aw8680x->dev, "rst request failed");
			return -RST_REGISTER_ERR;
		}
		AWLOGI(p_aw8680x->dev, "reset gpio request ok!!");
	} else {
		AWLOGE(p_aw8680x->dev, "reset gpio provided failed!");
		return -RST_REGISTER_ERR;
	}

	return AW_SUCCESS;
}

void aw8680x_irq_gpio_set(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;
	int32_t irq_flags = DATA_INIT;

	p_aw8680x->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (gpio_is_valid(p_aw8680x->irq_gpio)) {
		AWLOGI(p_aw8680x->dev, "irq gpio provided ok!!");
		ret = devm_gpio_request_one(p_aw8680x->dev, p_aw8680x->irq_gpio,
						GPIOF_DIR_IN, "aw8680x_int");
		if (ret) {
			AWLOGE(p_aw8680x->dev, "irq request failed!");
			devm_gpio_free(p_aw8680x->dev, p_aw8680x->irq_gpio);
			p_aw8680x->input_apply_flag = false;
			return;
		}
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(p_aw8680x->dev,
					gpio_to_irq(p_aw8680x->irq_gpio),
					NULL, aw8680x_irq, irq_flags,
					"aw8680x", p_aw8680x);
		if (ret != AW_SUCCESS) {
			AWLOGE(p_aw8680x->dev, "failed to request irq, ret:%d",
									ret);
			devm_gpio_free(p_aw8680x->dev, p_aw8680x->irq_gpio);
			p_aw8680x->input_apply_flag = false;
		} else {
			AWLOGI(p_aw8680x->dev, "irq gpio request ok!!");
			p_aw8680x->input_apply_flag = true;
			disable_irq(gpio_to_irq(p_aw8680x->irq_gpio));
		}
	} else {
		AWLOGE(p_aw8680x->dev, "irq gpio provided failed!");
		p_aw8680x->input_apply_flag = false;
	}

}

void aw8680x_wake_gpio_set(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;

	p_aw8680x->wake_gpio = of_get_named_gpio(np, "wake-gpio", 0);
	if (gpio_is_valid(p_aw8680x->wake_gpio)) {
		AWLOGI(p_aw8680x->dev, "wake gpio provided ok!!");
		ret = devm_gpio_request_one(p_aw8680x->dev,
						p_aw8680x->wake_gpio,
						GPIOF_OUT_INIT_LOW,
						"aw8680x_wake");
		if (ret) {
			AWLOGE(p_aw8680x->dev, "wake request failed!");
			devm_gpio_free(p_aw8680x->dev, p_aw8680x->wake_gpio);
			p_aw8680x->wake_flag = false;
		} else {
			AWLOGI(p_aw8680x->dev, "wake gpio request ok!!");
			p_aw8680x->wake_flag = true;
		}
	} else {
		AWLOGE(p_aw8680x->dev, "wake gpio provided failed!");
		p_aw8680x->wake_flag = false;
	}
}


static int32_t
aw8680x_parse_dt(struct aw8680x *p_aw8680x, struct device_node *np)
{
	int32_t ret = DATA_INIT;

	/* Use functions in kernel to get */
	ret = aw8680x_reset_gpio_set(p_aw8680x, np);
	if (ret != AW_SUCCESS) {
		AWLOGE(p_aw8680x->dev, "reset gpio request failed!");
		return -RST_REGISTER_ERR;
	}

	aw8680x_irq_gpio_set(p_aw8680x, np);
	aw8680x_wake_gpio_set(p_aw8680x, np);

	p_aw8680x->flash_boot_func =
			of_property_read_bool(np, "flash_boot_function_used");
	if (p_aw8680x->flash_boot_func == true)
		AWLOGI(p_aw8680x->dev, "flash boot update function is used!");
	else
		AWLOGI(p_aw8680x->dev, "flash boot update function is unused!");

	return AW_SUCCESS;
}

/******************************************************
 *
 * struct aw8680x init
 *
 ******************************************************/
void aw8680x_struct_init(struct aw8680x *p_aw8680x, struct i2c_client *i2c,
						const struct i2c_device_id *id)
{
	AWLOGI(&i2c->dev, "enter");

	p_aw8680x->dev = &i2c->dev;
	p_aw8680x->i2c = i2c;
	i2c_set_clientdata(i2c, p_aw8680x);
	dev_set_drvdata(&i2c->dev, p_aw8680x);
	g_aw8680x = p_aw8680x;
	p_aw8680x->adb_update_flash = NO_ADB;
	p_aw8680x->flash_update_ok = true;

}

void aw8680x_input_init(struct aw8680x *p_aw8680x)
{
	int32_t ret = DATA_INIT;

	AWLOGI(p_aw8680x->dev, "enter");

	p_aw8680x->input = input_allocate_device();
	if (!p_aw8680x->input) {
		input_free_device(p_aw8680x->input);
		AWLOGE(p_aw8680x->dev, "failed to allocate input device");
		return;
	}
	p_aw8680x->input->name = AW8680X_I2C_NAME;
	__set_bit(EV_KEY, p_aw8680x->input->evbit);
	__set_bit(EV_SYN, p_aw8680x->input->evbit);
	__set_bit(KEY_UP, p_aw8680x->input->keybit);

	ret = input_register_device(p_aw8680x->input);
	if (ret) {
		AWLOGE(p_aw8680x->dev, "failed to register input device: %s",
						dev_name(p_aw8680x->dev));
		input_unregister_device(p_aw8680x->input);
		input_free_device(p_aw8680x->input);
	}
	AWLOGI(p_aw8680x->dev, "input device regist OK!!");

}


/* In this function can do a series of initialization work */
static int32_t
aw8680x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int32_t ret = DATA_INIT;
	struct aw8680x *p_aw8680x = NULL;
	struct device_node *np = i2c->dev.of_node;

	AWLOGI(&i2c->dev, "enter");
	/* Determining the ability of the adapter */
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		AWLOGE(&i2c->dev, "check functionality faile");
		return -EIO;
	}
	/* Apply for memory for device structures */
	p_aw8680x = devm_kzalloc(&i2c->dev, sizeof(struct aw8680x), GFP_KERNEL);
	if (p_aw8680x == NULL)
		return -ENOMEM;

	aw8680x_struct_init(p_aw8680x, i2c, id);

	if (np) {
		ret = aw8680x_parse_dt(p_aw8680x, np);
		if (ret != AW_SUCCESS) {
			AWLOGE(&i2c->dev, "failed to parse device tree node");
			goto err_dt;
		}
	}

	ret = aw8680x_read_chipid(p_aw8680x);
	if (ret != DATA_INIT) {
		AWLOGE(&i2c->dev, "the soc not AW8680X");
		goto err_chipid;
	}

	/**********************************************************************
	*
	* flash_boot_version_get_flag :
	* flash boot version in soc whether or not the getting was successful
	*
	***********************************************************************/
	if (p_aw8680x->flash_boot_func == true) {
		ret = aw8680x_flash_boot_version_in_soc_get(p_aw8680x);
		p_aw8680x->flash_boot_version_get_flag = true;
		if (ret != AW_SUCCESS)
			p_aw8680x->flash_boot_version_get_flag = false;
		AWLOGI(p_aw8680x->dev, "the version of flash boot in soc is : V%x",
					p_aw8680x->flash_boot_version_in_soc);
	} else {
		p_aw8680x->flash_boot_version_get_flag = false;
		AWLOGI(p_aw8680x->dev, "the flash boot update function is unsupport!");
	}

	if (p_aw8680x->input_apply_flag == true)
		aw8680x_input_init(p_aw8680x);

	ret = sysfs_create_group(&i2c->dev.kobj, &aw8680x_attribute_group);
	if (ret < 0) {
		AWLOGE(&i2c->dev, "creating sysfs attr err !!!");
		goto err_attribute;
	}
	aw8680x_file_init(p_aw8680x);

	aw8680x_bin_init(p_aw8680x, AW8680X_BIN_INIT_DELAY);

	AWLOGI(p_aw8680x->dev, "probe completed successfully!");

	return AW_SUCCESS;
 err_attribute:
	sysfs_remove_group(&i2c->dev.kobj, &aw8680x_attribute_group);
 err_dt:
	devm_gpio_free(&i2c->dev, p_aw8680x->reset_gpio);
 err_chipid:
	devm_kfree(&i2c->dev, p_aw8680x);
	return ret;
}

static int aw8680x_i2c_remove(struct i2c_client *i2c)
{
	struct aw8680x *aw8680x = i2c_get_clientdata(i2c);

	input_free_device(aw8680x->input);
	input_unregister_device(aw8680x->input);
	sysfs_remove_group(&i2c->dev.kobj, &aw8680x_attribute_group);
	devm_kfree(&i2c->dev, aw8680x);

	return AW_SUCCESS;
}

/*****************************************************
 *
 * pm sleep
 *
 *****************************************************/
#ifdef CONFIG_PM_SLEEP
static int aw8680x_suspend(struct device *dev)
{
	pr_info("enter aw8680x_suspend\n");
	return 0;
}

static int aw8680x_resume(struct device *dev)
{
	pr_info("enter aw8680x_resume\n");
	return 0;
}

/* Sleep wake-up mechanism, in this function you can decide
 * what to do when the phone is in sleep wake-up state
 */
static SIMPLE_DEV_PM_OPS(aw8680x_pm_ops, aw8680x_suspend, aw8680x_resume);
#endif

static const struct i2c_device_id aw8680x_i2c_id[] = {
	{AW8680X_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw8680x_i2c_id);

/* Match device information in the device tree */
static const struct of_device_id aw8680x_dt_match[] = {
	{.compatible = "awinic,aw8680x"},
	{ },
};

static struct i2c_driver aw8680x_i2c_driver = {
	.driver = {
		   .name = AW8680X_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw8680x_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw8680x_pm_ops,
#endif
		   },
	.probe = aw8680x_i2c_probe,
	.remove = aw8680x_i2c_remove,
	.id_table = aw8680x_i2c_id,
};

static int __init aw8680x_i2c_init(void)
{
	int ret = 0;

	pr_info("aw8680x driver version %s\n", AW8680X_DRIVER_VERSION);
	/* Register the device driver on the i2c bus */
	ret = i2c_add_driver(&aw8680x_i2c_driver);
	if (ret) {
		pr_err("fail to add aw8680x device into i2c\n");
		return ret;
	}
	return 0;
}

/* Entry function */
module_init(aw8680x_i2c_init);

static void __exit aw8680x_i2c_exit(void)
{
	i2c_del_driver(&aw8680x_i2c_driver);
}

module_exit(aw8680x_i2c_exit);

MODULE_DESCRIPTION("AW8680X Sensor Driver");
MODULE_LICENSE("GPL v2");
