/*
* Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2020 .
* All rights reserved.
* Description: Communication protocol fun related header file
*/
#include "aw_protocol_config.h"

#ifdef SOC_PROTOCOL_VALID
#include "aw_protocol_fun.h"
#include "aw_protocol_map.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

AW_U8 aw_set_u32_fun(AW_U8 *u8_addr, AW_U32 u32_data)
{
#ifdef POINTER_CHECK
		if (u8_addr == AW_NULL)
			return AW_FAIL;
#endif

	u8_addr[0] = GET_0BYT(u32_data);
	u8_addr[1] = GET_1BYT(u32_data);
	u8_addr[2] = GET_2BYT(u32_data);
	u8_addr[3] = GET_3BYT(u32_data);

	return AW_OK;
}

AW_U32 aw_get_u32_fun(AW_U8 *u8_addr)
{
#ifdef POINTER_CHECK
	if (u8_addr == AW_NULL)
		return AW_FAIL;
#endif

	return (AW_U32)((u8_addr[0]<<0) + (u8_addr[1]<<8) +
				(u8_addr[2]<<16) + (u8_addr[3]<<24));
}

static AW_U16 aw_get_u16_fun(AW_U8 *u8_addr)
{
#ifdef POINTER_CHECK
	if (u8_addr == AW_NULL)
		return AW_FAIL;
#endif

	return (AW_U16)((u8_addr[1] << 8) | u8_addr[0]);
}

AW_U8 aw_set_tx_data_ack(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
								AW_U8 ack_data)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l2_s.ack = ack_data;
	return AW_OK;
}

AW_U8 aw_set_tx_data_error(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
								AW_U8 err_data)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l3_data[ERROR_LOCATION_DATA] =
								     err_data;

	return AW_OK;
}

AW_U8 aw_set_tx_location_fun(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
								AW_U32 boot_num)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	aw_set_u32_fun(&p_aw_protocol_tx_data_s->protocol_l3_data
						[FUN_LOCATION_DATA], boot_num);

	return AW_OK;
}

AW_U32 aw_get_rx_write_data_addr(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return aw_get_u32_fun(&p_aw_protocol_rx_data_s->protocol_l3_data
							[WRITE_DATA_ADDR]);
}

AW_U8 *aw_get_rx_write_use_data(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_NULL;
#endif

	return &p_aw_protocol_rx_data_s->protocol_l3_data[WRITE_USE_DATA];
}

AW_U8 aw_set_tx_module_id(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U8 module_id_num)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l2_s.module_id = module_id_num;

	return AW_OK;
}

AW_U8 aw_get_rx_module_id(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return p_aw_protocol_rx_data_s->protocol_l2_s.module_id;
}

AW_U8 aw_set_tx_event_id(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U8 event_id_data)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l2_s.event_id = event_id_data;

	return AW_OK;
}

AW_U8 aw_get_rx_event_id(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return p_aw_protocol_rx_data_s->protocol_l2_s.event_id;
}

AW_U16 aw_get_rx_payload_length(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return GET_16_DATA(p_aw_protocol_rx_data_s->protocol_l2_s.
				payload_length_h, p_aw_protocol_rx_data_s->
				protocol_l2_s.payload_length_l);
}

AW_U8 aw_get_rx_check_data(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return p_aw_protocol_rx_data_s->protocol_l2_s.check_data;
}

AW_U16 aw_get_rx_read_erase_length(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return aw_get_u16_fun(&p_aw_protocol_rx_data_s->protocol_l3_data
								[LENGTH_DATA]);
}

AW_U32 aw_get_rx_read_data_addr(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return aw_get_u32_fun(&p_aw_protocol_rx_data_s->protocol_l3_data
							[READ_DATA_ADDR]);
}

AW_U8 aw_set_tx_read_data_addr(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U32 data_addr)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	aw_set_u32_fun(&p_aw_protocol_tx_data_s->protocol_l3_data
					[READ_DATA_ADDR_ACK], data_addr);

	return AW_OK;
}

AW_U8 *aw_get_tx_send_data_buff(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL)
		return AW_NULL;
#endif

	return &p_aw_protocol_tx_data_s->protocol_l3_data[READ_SEND_DATA];
}

AW_U32 aw_get_rx_end_addr(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s)
{
#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	return aw_get_u32_fun(&p_aw_protocol_rx_data_s->protocol_l3_data
								[END_ADDR]);
}

AW_U32 aw_set_err_id_fun(PROTOCOL_DATA_S *p_aw_protocol_rx_data_s,
				PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
				ADDRESS_ID_E aw_adress)
{
#ifdef POINTER_CHECK
	if ((p_aw_protocol_rx_data_s == AW_NULL) ||
	    (p_aw_protocol_tx_data_s == AW_NULL))
		return AW_FAIL;
#endif

	p_aw_protocol_tx_data_s->protocol_l3_data[ERR_ACK_MODULE] =
			p_aw_protocol_rx_data_s->protocol_l2_s.module_id;
	p_aw_protocol_tx_data_s->protocol_l3_data[ERR_ACK_EVNENT_LEN] =
				p_aw_protocol_rx_data_s->protocol_l2_s.event_id;
	p_aw_protocol_tx_data_s->protocol_l3_data[ERR_ACK_CHIP_LEN] = aw_adress;

	return AW_OK;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* SOC_PROTOCOL_VALID */


