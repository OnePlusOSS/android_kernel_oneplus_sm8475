#ifndef __AW8680X_H__
#define __AW8680X_H__

#define AWINIC_DEBUG
#ifdef AWINIC_DEBUG
#define AWLOGD(dev, format, arg...) \
	do {\
		 dev_printk(KERN_ERR, dev, \
			"[%s:%d] "format"\n", __func__, __LINE__, ##arg);\
	} while (0)

#define AWLOGI(dev, format, arg...) \
	do {\
		dev_printk(KERN_ERR, dev, \
			"[%s:%d] "format"\n", __func__, __LINE__, ##arg);\
	} while (0)

#define AWLOGE(dev, format, arg...) \
	do {\
		 dev_printk(KERN_ERR, dev, \
			"[%s:%d] "format"\n", __func__, __LINE__, ##arg);\
	} while (0)
#else
#define AWLOGD(dev, format, arg...)
#define AWLOGI(dev, format, arg...)
#define AWLOGE(dev, format, arg...)
#endif

/*********************************************************
 *
 * struct
 *
 ********************************************************/
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

#define SOC_APP_DATA_TYPE			0x21
#define PC_POINT_ROM_BOOT			0x00000001
#define PC_POINT_FLASH_BOOT			0x00010002
#define PC_POINT_SRAM				0x00010003

#define ERASE_BYTE_MAX				512
#define WRITE_FLASH_MAX				64
#define READ_FLASH_MAX				64
#define WRITE_SRAM_MAX				(64 * 3)

#define REG_ADDR				0x01
#define SOC_ADDR				0x00000000
#define SOC_DATA_LEN				0x0000
#define SOC_READ_LEN				0x0000

#define KEY_DATA_ADDR				0x12
#define REG_SCAN_MODE_SWITCH_ADDR		0x56
#define REGVAL_LEN_ONE_byte			1
#define RESET_INIT_TIME				5
#define CHIP_INIT_TIME				15
#define JUMP_INIT_TIME				10
#define CONNECT_RETRY_TIME			3
#define READ_CHIPID_RETRY_TIME			3
#define REG_STAY_UBOOT				0x01
#define AW8680X_STAY_UBOOT			1

#define FLASH_APP_VERSION_GET_TIME		50
#define SRAM_INIT_TIME				20
#define FLASH_APP_INIT_TIME			20
#define FLASH_BOOT_INIT_TIME			10

/* register */
#define RAW_DATA_NUM				3
#define FORCE_DATA_NUM				3
#define BASE_DATA_NUM				3
#define DIFF_DATA_NUM				3
#define ADC_DATA_NUM				4
#define RAW_DATA_LEN				6
#define FORCE_DATA_LEN				6
#define BASE_DATA_LEN				6
#define DIFF_DATA_LEN				6
#define ADC_DATA_LEN				8
#define RAW_DATA_ADDR				0x19
#define FORCE_DATA_ADDR				0x20
#define BASE_DATA_ADDR				0x12
#define DIFF_DATA_ADDR				0x13
#define ADC_DATA_ADDR				0x14
#define DATA_INIT				(0)

/* about i2c msg */
#define ONE_MSG_NUM				1
#define TWO_MSG_NUM				2

/* chipid */
#define AW86801					0x86801
#define AW86802					0x86802
#define AW86803					0x86803
#define AW86802A				0x86802A

/* flash and sram address division */
#define FLASH_BOOT_BASE_ADDR			0x01000000
#define FLASH_APP_BASE_ADDR			0x01001000
#define FLASH_MAX_ADDR				0x01010000
#define SRAM_BASE_ADDR				0x20001000
#define SRAM_MAX_ADDR				0x20002000
#define ERASE_FLASH_BOOT_SIZE			8

#define NO_ADB					0
#define FLASH_APP_IN_SOC_BLANK			-107

enum gpio_level_signal {
	LOW_LEVEL,
	HIGH_LEVEL,
};

enum adb_update_flash {
	ADB_UPDATE_FLASH_BOOT = 1,
	ADB_UPDATE_FLASH_APP,
};

enum aw8680x_i2c_flag {
	I2C_WRITE_FLAG,
	I2C_READ_FLAG,
};

enum return_flag_enum {
	AW_SUCCESS,
	ERR_FLAG,
	CHIPID_ERR,
	ACK_ERR,
	ERR_JUMP,
	NOT_NEED_UPDATE,
	RST_REGISTER_ERR,
	WAKE_REGISTER_ERR,
	IRQ_REGISTER_ERR,
	IRQ_THREAD_ERR,
	ERR_CREAT_SYS,
	SRAM_BIN_FAILED,
	JUMP_BOOT_FAILED,
	FLAH_BOOT_BIN_ERR,
	FLAH_APP_BIN_ERR,
};


enum report_mode_enum {
	IRQ_MODE_SET = true,
	POLLING_MODE_SET = false,
};

enum scan_mode_switch_enum {
	SWITCH = 1,
	HIGH_SPEED = 2,
	LOW_SPEED = 3,
	POWER_OFF = 4,
};

enum updata_enum {
	AW8680X_FLASH_NO_UPDATE,
	AW8680X_FLASH_BOOT_UPDATE,
	AW8680X_FLASH_APP_UPDATE,
};

struct data_container {
	unsigned int len;
	unsigned char data[];
};

struct aw8680x_sensor_info {
	unsigned char key_data;
	unsigned short adc_data[ADC_DATA_NUM];
	unsigned short raw_data[RAW_DATA_NUM];
	unsigned short force_data[FORCE_DATA_NUM];
	unsigned short base_data[BASE_DATA_NUM];
	unsigned short diff_data[DIFF_DATA_NUM];
};

struct aw8680x {
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;
	struct delayed_work bin_work;
	struct aw_bin *sram_bin;
	struct aw_bin *flash_app_bin;
	struct aw_bin *flash_boot_bin;
	struct aw8680x_sensor_info info;
	struct task_struct *thread;
	uint32_t adb_update_flash;
	bool flash_boot_func;
	bool input_apply_flag;
	bool wake_flag;
	bool jump_flash_app_flag;
	bool flash_boot_version_get_flag;
	bool flash_app_version_get_flag;

	GUI_TO_SOC_S p_gui_data_s;
	unsigned char p_protocol_tx_data[PROTOCOL_TOTAL_LEN];
	unsigned char p_protocol_rx_data[PROTOCOL_TOTAL_LEN];

	char read_data[256];
	unsigned char irq_key_data;
	unsigned char module_id;
	unsigned char reg_addr;
	unsigned short read_len;

	unsigned int flash_app_addr;
	unsigned int flash_boot_addr;
	unsigned int sram_addr;

	uint32_t pc_location;
	uint32_t pc_point_flash_app;

	uint32_t flash_boot_version_in_soc;
	uint32_t flash_app_version_in_soc;
	uint32_t flash_boot_version_in_bin;
	uint32_t flash_app_version_in_bin;

	uint32_t flash_app_update_flag;
	uint32_t flash_boot_update_flag;

	bool flash_update_ok;

	uint32_t chipid_data;

	int32_t irq_gpio;
	int32_t reset_gpio;
	int32_t wake_gpio;
	int32_t ack_flag;
};

#endif
