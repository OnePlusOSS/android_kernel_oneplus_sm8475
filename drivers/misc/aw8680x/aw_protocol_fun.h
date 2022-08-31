/*
* Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
* All rights reserved.
* Description: Communication protocol fun related header file
*/
#ifndef __AW_PROTOCOL_FUN_H
#define __AW_PROTOCOL_FUN_H
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define GET_3BYT(data) (AW_U8)((data)>>24)
#define GET_2BYT(data) (AW_U8)((data)>>16)
#define GET_1BYT(data) (AW_U8)((data)>>8)
#define GET_0BYT(data) (AW_U8)((data)>>0)

#define CLEAN_HEAD_BYTE			((AW_U32)0x00FFFFFF)
#define CHECK_ACK_LEN			(0x01)
#define HANDSHAKE_ACK_LEN		(0x05)
#define ID_NO_RESPOND_LEN		(0x04)
#define AW_LOCATION_UBOOT		((AW_U32)0x00000001)
#define AW_FLASH_BOOT			((AW_U32)0x00010002)
#define AW_RAM_BOOT				((AW_U32)0x00010003)
#define READ_ACK_LENGTH			((AW_U16)0x05) /* flags and addresses */

#define ERROR_LOCATION_DATA		(0)
#define FUN_LOCATION_DATA		(1)
#define WRITE_DATA_ADDR			(0)
#define WRITE_USE_DATA			(4)
#define LENGTH_DATA				(0)
#define LENGTH_DATA_H			(1)
#define READ_DATA_ADDR			(2)
#define READ_DATA_ADDR_ACK		(1)
#define READ_SEND_DATA			(5)
#define END_ADDR				(0)
/* IF module or evnet id err ,ack moudel_id location length */
#define ERR_ACK_MODULE			(1)
/* IF module or evnet id err ,ack evenet_id location length */
#define ERR_ACK_EVNENT_LEN		(2)
/* IF module or evnet id err ,AW chip location length */
#define ERR_ACK_CHIP_LEN		(3)

AW_U8 aw_set_u32_fun(AW_U8 *u8_addr, AW_U32 u32_data);
AW_U32 aw_get_u32_fun(AW_U8 *u8_addr);
AW_U8 aw_get_rx_module_id(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U8 aw_set_tx_module_id(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U8 module_id_num);
AW_U8 aw_set_tx_data_error(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U8 err_data);
AW_U8 aw_set_tx_location_fun(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U32 boot_num);
AW_U8 aw_get_rx_event_id(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U8 aw_set_tx_event_id(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U8 event_id_data);
AW_U32 aw_get_rx_write_data_addr(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U16 aw_get_rx_payload_length(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U8 aw_get_rx_check_data(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U8 *aw_get_rx_write_use_data(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U16 aw_get_rx_read_erase_length(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U32 aw_get_rx_read_data_addr(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U8 *aw_get_tx_send_data_buff(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s);
AW_U8 aw_set_tx_read_data_addr(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U32 data_addr);
AW_U32 aw_get_rx_end_addr(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s);
AW_U8 aw_set_tx_data_ack(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U8 ack_data);
AW_U32 aw_set_err_id_fun(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s,
	PROTOCOL_DATA_S *p_aw_protocol_tx_data_s, ADDRESS_ID_E aw_adress);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
