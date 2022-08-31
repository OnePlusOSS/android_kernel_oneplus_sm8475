/*
* Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
* All rights reserved.
* Description: Communication protocol map related header file
*/
#ifndef __AW_PROTOCOL_MAP_H
#define __AW_PROTOCOL_MAP_H
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define GET_16_DATA(x, y) ((AW_U16)(((x) << 8) | (y)))
#define ERASE_SCTOR_ID (0x0321)
#define ERASE_CHIP_ID (0x0323)

#ifdef AW_PROTOCOL_MCU
#define CONNECT_NUM				(0)
#define PROTOCOL_VERSION_NUM	(18)
#define VERSION_INFOR_NUM		(20)
#define CHIP_ID_NUM				(24)
#define CHIP_DATE_NUM			(26)
#define RAM_READ_NUM			(4)
#define FLASH_READ_NUM			(8)
#define UI_IRQ_NUM				(22)
#define DATA_LEN_H				(1)
#define DATA_LEN_L				(0)
#define DATA_ACK_LEN			(14)
#define DATA_ACK_ERR_LEN		(10)

AW_U8 aw_set_ui_read_data_len(GUI_TO_SOC_S *p_gui_data_s);
#endif /* AW_PROTOCOL_MCU */
AW_U8 aw_check_id_fun(AW_U8 module_id, AW_U8 event_id);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
