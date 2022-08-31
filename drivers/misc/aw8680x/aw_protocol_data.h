/*
* Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
* All rights reserved.
* Description: Communication protocol data related header file
*/
#ifndef __AW_PROTOCOL_DATA_H
#define __AW_PROTOCOL_DATA_H
#include "aw_type.h"
#include "aw_protocol_config.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define PROTOCOL_TOTAL_LEN		(256)
#define PROTOCOL_L1_S_LEN		(sizeof(PROTOCOL_L1_S))
#define PROTOCOL_L2_S_LEN		(sizeof(PROTOCOL_L2_S))
#define IIC_L3_DATA_LEN			(PROTOCOL_TOTAL_LEN - PROTOCOL_L1_S_LEN - PROTOCOL_L2_S_LEN)
#define PROTOCOL_S_LEN			(sizeof(PROTOCOL_DATA_S))
#define CHECK_PROTOCOL_LEN		(PROTOCOL_L1_S_LEN + PROTOCOL_L2_S_LEN - 1)

enum check_ack_enum {
	AW_ACK_BUSY = 0x00,
	AW_ACK_FREE = 0x01,
};
typedef enum check_ack_enum AW_ACK_E;

enum version_enum {
	AW_VERSION_1 = 0x01, /* protocol version */
	AW_VERSION_2 = 0x02,
	AW_VERSION_3 = 0x03,
};
typedef enum version_enum AW_VERSION_E;

enum id_enum {
	PC_ADDRESS_ID  = 0x01,
	MCU_ADDRESS_ID = 0x02,
	AP_ADDRESS_ID  = 0x04,
	AW_ADDRESS_ID  = 0x08,
};
typedef enum id_enum ADDRESS_ID_E;

enum module_enum {
	AW_ID_FREE			= 0X00,
	HANDSHAKE_ID			= 0x01,
	RAM_ID				= 0x02,
	FLASH_ID			= 0x03,
	END_ID				= 0x04,
	UI_IRQ_ID			= 0x05,
	AW_MODULE_ID_ERR		= 0Xff,
};
typedef enum module_enum MODULE_E;

enum connect_enum {
	CONNECT_ID = 0x01,
	CONNECT_ACK_ID = 0x02,
	MOUDLE_EVNET_NO_RESPOND = 0x08,
	PROTOCOL_ID = 0x11,
	PROTOCOL_ACK_ID = 0x12,
	VERSION_ID = 0x21,
	VERSION_ACK_ID = 0x22,
	CHIP_ID = 0x31,
	CHIP_ID_ACK = 0x32,
	CHIP_DATE = 0x33,
	CHIP_DATE_ACK = 0x34,
	AW_EVENT_ID_ERR = 0Xff,
};
typedef enum connect_enum HANDSHAKE_EVENT_E;

enum ram_enum {
	RAM_WRITE_ID	 = 0x01,
	RAM_WRITE_ACK_ID = 0x02,
	RAM_READ_ID		 = 0x11,
	RAM_READ_ACK_ID	 = 0x12,
};
typedef enum ram_enum RAM_EVENT_E;

enum flash_enum {
	FLASH_WRITE_ID		 = 0x01,
	FLASH_WRITE_ACK_ID	 = 0x02,
	FLASH_READ_ID		  = 0x11,
	FLASH_READ_ACK_ID	  = 0x12,
	FLASH_ERASE_ID		 = 0x21,
	FLASH_ERASE_ACK_ID	 = 0x22,
	FLASH_ERASE_CHIP_ID	= 0X23,
	FLASH_ERASE_CHIP_ACK_ID = 0X24,
};
typedef enum flash_enum FLASH_EVENT_E;

enum end_enum {
	RAM_JUMP_ID		= 0x01,
	RAM_JUMP_ACK_ID		= 0x02,
	FLASH_JUMP_ID		= 0x11,
	FLASH_JUMP_ACK_ID	= 0x12,
	ROM_JUMP_ID		= 0x21,
	ROM_JUMP_ACK_ID		= 0x22,
};
typedef enum end_enum END_EVENT_E;

enum ui_irq_enum {
	READ_INT_STA_ID			= 0x01,
	READ_INT_STA_ACK_ID		= 0x02,
};
typedef enum ui_irq_enum UI_IRQ_E;

enum check_flag_enum {
	AW_FLAG_OK = 0x00,
	AW_FLAG_FAIL = 0x01,
	AW_CHECK_HEADER_ERR = 0x02,
	AW_VERSION_ERR = 0x03,
	AW_ADDRESS_ID_ERR = 0x04,
	AW_CHECK_DATA_ERR = 0x05,
	AW_CHECK_ID_ERR = 0x06,
	AW_ADDRESS_ERR = 0x07,
	AW_HANDSHAKE_ERR = 0x10,
	AW_RAM_WRITE_ERR = 0x20,
	AW_RAM_READ_ERR = 0x21,
	AW_FLASH_WRITE_ERR = 0x30,
	AW_FLASH_READ_ERR = 0x31,
	AW_FLASH_SECTOR_ERR = 0x32,
	AW_FLASH_CHIP_ERR = 0x33,
	AW_END_ERR = 0x40,
	AW_TIME_OUT_ERR = 0x41,
	AW_EXCEPTION_ERR = 0xff,
};
typedef enum check_flag_enum AW_CHECK_FLAG_E;

struct protocol_l1_struct {
	AW_U8 check_header;
	AW_U8 version;
	AW_U8 adress;
};
typedef struct protocol_l1_struct PROTOCOL_L1_S;

struct protocol_l2_struct {
	AW_U8 module_id;
	AW_U8 event_id;
	AW_U8 payload_length_l;
	AW_U8 payload_length_h;
	AW_U8 ack;
	AW_U8 check_data;
};
typedef struct protocol_l2_struct PROTOCOL_L2_S;

struct protocol_data_struct {
	PROTOCOL_L1_S protocol_l1_s;
	PROTOCOL_L2_S protocol_l2_s;
	AW_U8 protocol_l3_data[IIC_L3_DATA_LEN];
};
typedef struct protocol_data_struct PROTOCOL_DATA_S;

struct data_buff_struct {
	AW_U8 dest_adr;
	AW_U8 src_adr;
} ;
typedef struct data_buff_struct COM_DATA_BUFF_S;

#ifdef AW_PROTOCOL_MCU
struct gui_to_soc_struct {
	AW_U16 version_num; /* Effective data length */
	AW_U16 soc_data_len; /* Effective data length */
	AW_U16 ui_rd_data_len; /* Data length to be read after the upper computer finishes writing data */
	AW_U16 read_len; /* Read addr data length */
	AW_U32 addr; /* Address where writes data */
	AW_U8 device_commu;
	AW_U8 device_addr;
	AW_U8 dest_adr; /* Destination address means: to whom? */
	AW_U8 src_adr; /* source addres means:form where? */
	AW_U8 module_id; /* module id */
	AW_U8 event_id; /* evnet id */
	AW_U8 err_flag; /* Operation status of lower computer */
	AW_U8 reserved0; /* reserved */
	AW_U8 reserved1; /* reserved */
	AW_U8 soc_data[IIC_L3_DATA_LEN]; /* Effective data */
};
typedef struct gui_to_soc_struct GUI_TO_SOC_S;
#endif /* AW_PROTOCOL_MCU */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
