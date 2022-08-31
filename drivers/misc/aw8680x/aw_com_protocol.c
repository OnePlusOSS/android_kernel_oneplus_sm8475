/*
* Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2020 .
* All rights reserved.
* Description: communication protocol related functions
*/
#include "aw_protocol_config.h"

#ifdef SOC_PROTOCOL_VALID
#include "aw_protocol_map.h"
#include "aw_com_protocol.h"
#include "aw_protocol_fun.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

static COM_DATA_BUFF_S g_aw_com_data;

static COM_DATA_BUFF_S *aw_get_com_data_struct(void)
{
	return &g_aw_com_data;
}

AW_U8 check_sum(AW_U8 *buf, AW_U16 len)
{
	AW_U8 sum_data = 0;
	AW_U16 i = 0;

#ifdef POINTER_CHECK
	if (buf == AW_NULL)
		return AW_FAIL;
#endif

	for (i = 0; i < len; i++)
		sum_data += buf[i];

	return sum_data;
}

static AW_U8 aw_parsing_protocol_header(PROTOCOL_DATA_S
						*p_aw_protocol_rx_data_s)
{
	COM_DATA_BUFF_S *p_aw_com_data = AW_NULL;

#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FAIL;
#endif

	p_aw_com_data = aw_get_com_data_struct();
	p_aw_com_data->dest_adr = GET_HIGH_4BIT(p_aw_protocol_rx_data_s->
							protocol_l1_s.adress);
	p_aw_com_data->src_adr = GET_LOW_4BIT(p_aw_protocol_rx_data_s->
							protocol_l1_s.adress);

	return AW_OK;
}

AW_CHECK_FLAG_E check_protocol_header_data(PROTOCOL_DATA_S
				*p_aw_protocol_rx_data_s, ADDRESS_ID_E aw_id_e)
{
	AW_U8 check_sum_data = 0;
	AW_U8 check_id_num = 0;
	AW_U16 payload_length_data = 0;
	AW_U8 *p_protocol_rx_l3 = AW_NULL;
	PROTOCOL_L1_S *p_protocol_rx_l1_s = AW_NULL;
	PROTOCOL_L2_S *p_protocol_rx_l2_s = AW_NULL;
	COM_DATA_BUFF_S *p_aw_com_data = AW_NULL;

#ifdef POINTER_CHECK
	if (p_aw_protocol_rx_data_s == AW_NULL)
		return AW_FLAG_FAIL;
#endif

	p_protocol_rx_l3 = p_aw_protocol_rx_data_s->protocol_l3_data;
	p_protocol_rx_l1_s = &p_aw_protocol_rx_data_s->protocol_l1_s;
	p_protocol_rx_l2_s = &p_aw_protocol_rx_data_s->protocol_l2_s;
	p_aw_com_data = aw_get_com_data_struct();

	aw_parsing_protocol_header(p_aw_protocol_rx_data_s);

	payload_length_data = GET_16_DATA(p_protocol_rx_l2_s->payload_length_h,
					p_protocol_rx_l2_s->payload_length_l);

	check_id_num = aw_check_id_fun(p_protocol_rx_l2_s->module_id,
						p_protocol_rx_l2_s->event_id);
	check_sum_data = check_sum(&(p_protocol_rx_l1_s->version),
							CHECK_PROTOCOL_LEN);
	if (check_sum_data != p_protocol_rx_l1_s->check_header) {
		return AW_CHECK_HEADER_ERR;
	} else if (p_protocol_rx_l1_s->version != AW_VERSION_1) {
		return AW_VERSION_ERR;
	} else if (p_aw_com_data->dest_adr != aw_id_e) {
		return AW_ADDRESS_ID_ERR;
	} else if (check_id_num == AW_FAIL) {
		return AW_CHECK_ID_ERR;
	} else {
		check_sum_data = check_sum(p_protocol_rx_l3,
							payload_length_data);
		if ((check_sum_data != p_protocol_rx_l2_s->check_data))
			return AW_CHECK_DATA_ERR;
	}

	return AW_FLAG_OK;
}

AW_U8 aw_set_protocol_header_data(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
								AW_U16 data_len)
{
	AW_U8 *p_protocol_tx_l3 = AW_NULL;
	PROTOCOL_L1_S *p_protocol_tx_l1_s = AW_NULL;
	PROTOCOL_L2_S *p_protocol_tx_l2_s = AW_NULL;
	COM_DATA_BUFF_S *p_aw_com_data = AW_NULL;

#ifdef POINTER_CHECK
	if (p_aw_protocol_tx_data_s == AW_NULL) {
		return AW_FAIL;
	}
#endif

	p_protocol_tx_l3 = p_aw_protocol_tx_data_s->protocol_l3_data;
	p_protocol_tx_l1_s = &p_aw_protocol_tx_data_s->protocol_l1_s;
	p_protocol_tx_l2_s = &p_aw_protocol_tx_data_s->protocol_l2_s;
	p_aw_com_data = aw_get_com_data_struct();

	p_protocol_tx_l1_s->version = AW_VERSION_1;
	p_protocol_tx_l1_s->adress = (AW_U8)(p_aw_com_data->dest_adr |
						p_aw_com_data->src_adr<<4);
	p_protocol_tx_l2_s->payload_length_l = GET_0BYT(data_len);
	p_protocol_tx_l2_s->payload_length_h = GET_1BYT(data_len);
	p_protocol_tx_l2_s->ack = AW_ACK_FREE;
	p_protocol_tx_l2_s->check_data = check_sum(p_protocol_tx_l3, data_len);
	p_protocol_tx_l1_s->check_header = check_sum(&(p_protocol_tx_l1_s->
						version), CHECK_PROTOCOL_LEN);

	return AW_OK;
}

#ifdef AW_PROTOCOL_MCU
/**
  * @brief  L3 layer conversion function from UI data to SOC data
  * @param  p_aw_protocol_data_s,p_aw_protocol_data_s,data_len
  * @retval AW_U8 data
  */
static AW_U8 aw_gui_exchange_buff(PROTOCOL_DATA_S *p_aw_protocol_data_s,
						GUI_TO_SOC_S *p_gui_data_s,
						AW_U16 *data_len)
{
	AW_U16 i = 0;

#ifdef POINTER_CHECK
	if ((p_aw_protocol_data_s == AW_NULL) || (p_gui_data_s == AW_NULL))
		return AW_FLAG_FAIL;
#endif
	/* If there is data length in L3 layer */
	if (p_gui_data_s->read_len != 0) {
		/* Populate data according to protocol */
		p_aw_protocol_data_s->protocol_l3_data[LENGTH_DATA] =
					GET_0BYT(p_gui_data_s->read_len);
		p_aw_protocol_data_s->protocol_l3_data[LENGTH_DATA_H] =
					GET_1BYT(p_gui_data_s->read_len);
		/* Set L3 layer data */
		aw_set_u32_fun(&p_aw_protocol_data_s->protocol_l3_data[
					READ_DATA_ADDR], p_gui_data_s->addr);
		/* Calculate L3 data length */
		*data_len = UI_LENGTH_LEN + UI_ADDR_LEN;
	} else {
		if (p_gui_data_s->soc_data_len == 0) {
			if (p_gui_data_s->addr != 0) {
				/* When there is only address, the data length
				 * of L3 layer is the address length
				 */
				aw_set_u32_fun(p_aw_protocol_data_s->
					protocol_l3_data, p_gui_data_s->addr);
				*data_len = UI_ADDR_LEN;
			} else {
				/* When both data and address are 0,
				 * L3 layer length is 0
				 */
				*data_len = p_gui_data_s->soc_data_len;
			}
		} else {
			/* Set L3 layer data */
			aw_set_u32_fun(p_aw_protocol_data_s->protocol_l3_data,
							p_gui_data_s->addr);
			for (i = 0; i < p_gui_data_s->soc_data_len; i++) {
				p_aw_protocol_data_s->protocol_l3_data[i +
				      UI_ADDR_LEN] = p_gui_data_s->soc_data[i];
			}
			/* Set L3 layer data length */
			*data_len = p_gui_data_s->soc_data_len + UI_ADDR_LEN;
		}
	}

	return AW_OK;
}

/**
  * @brief  UI to SOC packet function
  * @param  p_gui_data_s,p_aw_protocol_tx_data
  * @retval AW_U8 data
  */
AW_U8 aw_soc_protocol_pack(GUI_TO_SOC_S *p_gui_data_s,
						AW_U8 *p_aw_protocol_tx_data)
{
	AW_U16 aw_data_len = 0;
	PROTOCOL_DATA_S *p_aw_protocol_tx_data_s = AW_NULL;
	PROTOCOL_L2_S *p_protocol_tx_l2_s = AW_NULL;
	COM_DATA_BUFF_S *p_aw_com_data = AW_NULL;

#ifdef POINTER_CHECK
	if ((p_gui_data_s == AW_NULL) || (p_aw_protocol_tx_data == AW_NULL))
		return AW_FAIL;
#endif

	/* Strongly convert array to struct type */
	p_aw_protocol_tx_data_s = (PROTOCOL_DATA_S *)p_aw_protocol_tx_data;
	p_protocol_tx_l2_s = &p_aw_protocol_tx_data_s->protocol_l2_s;

	/* set dest addr and src addr */
	p_aw_com_data = aw_get_com_data_struct();
	p_aw_com_data->dest_adr = p_gui_data_s->src_adr;
	p_aw_com_data->src_adr = p_gui_data_s->dest_adr;

	/* set module and event id */
	p_protocol_tx_l2_s->module_id = p_gui_data_s->module_id;
	p_protocol_tx_l2_s->event_id = p_gui_data_s->event_id;

	/* Get UI read IIC ack data length */
	aw_set_ui_read_data_len(p_gui_data_s);
	/* L3 layer conversion function from UI data to SOC data */
	aw_gui_exchange_buff(p_aw_protocol_tx_data_s, p_gui_data_s,
								&aw_data_len);
	/* Calculate the current total data length */
	p_gui_data_s->soc_data_len = aw_data_len + UI_L1_L2_LEN;
	/* Data check and combination */
	aw_set_protocol_header_data(p_aw_protocol_tx_data_s, aw_data_len);

	return AW_OK;
}

/**
  * @brief  L3 layer data extraction function
  * @param  p_gui_data_s,p_aw_protocol_tx_data,data_len
  * @retval AW_U8 data
  */
static AW_U8 aw_soc_exchange_buff(GUI_TO_SOC_S *p_gui_data_s,
			PROTOCOL_DATA_S *p_aw_protocol_data_s, AW_U16 data_len)
{
	AW_U16 i = 0;
	AW_U32 ui_data = 0;

#ifdef POINTER_CHECK
	if ((p_aw_protocol_data_s == AW_NULL) || (p_gui_data_s == AW_NULL) ||
							   (data_len == 0))
		return AW_FLAG_FAIL;
#endif

	if (data_len > IIC_L3_DATA_LEN)
		data_len = IIC_L3_DATA_LEN;

	if (data_len == CHECK_ACK_LEN) {
		/* No useful data */
		p_gui_data_s->soc_data_len = 0;
		/* Data in handshake module */
	} else if (data_len == ID_NO_RESPOND_LEN) {
		/* set data length */
		p_gui_data_s->soc_data_len = NO_RESPOND_LEN;
		/* Put the data into the UI structure */
		p_gui_data_s->soc_data[0] = p_aw_protocol_data_s->
					protocol_l3_data[ERR_ACK_MODULE];
		p_gui_data_s->soc_data[1] = p_aw_protocol_data_s->
					protocol_l3_data[ERR_ACK_EVNENT_LEN];
		p_gui_data_s->soc_data[2] = p_aw_protocol_data_s->
					protocol_l3_data[ERR_ACK_CHIP_LEN];
		/* Data in handshake module */
	} else if (data_len == HANDSHAKE_ACK_LEN) {
		/* set data length */
		p_gui_data_s->soc_data_len = UI_ADDR_LEN;
		/* Put the data into the UI structure */
		ui_data = aw_get_u32_fun(&p_aw_protocol_data_s->
					protocol_l3_data[FUN_LOCATION_DATA]);
		aw_set_u32_fun(p_gui_data_s->soc_data, ui_data);
	} else if (data_len >= READ_ACK_LENGTH) {
		/* Subtract the length of address and flag */
		data_len = data_len - READ_ACK_LENGTH;
		/* set data length */
		p_gui_data_s->soc_data_len = data_len;
		/* set addr */
		p_gui_data_s->addr = aw_get_u32_fun(&p_aw_protocol_data_s->
					protocol_l3_data[READ_DATA_ADDR_ACK]);
		/* set data */
		for (i = 0; i < data_len; i++) {
			p_gui_data_s->soc_data[i] = p_aw_protocol_data_s->
					protocol_l3_data[i + READ_ACK_LENGTH];
		}
	} else {
		return AW_FAIL;
	}

	return AW_OK;
}

/**
  * @brief  SOC to UI data conversion function
  * @param  p_gui_data_s,p_aw_protocol_tx_data,aw_id_e
  * @retval AW_CHECK_FLAG_E flag
  */
AW_CHECK_FLAG_E aw_soc_protoco_unpack(GUI_TO_SOC_S *p_gui_data_s,
						AW_U8 *p_aw_protocol_rx_data)
{
	AW_U16 aw_data_len = 0;
	AW_CHECK_FLAG_E sta_flag = AW_FLAG_OK;
	PROTOCOL_DATA_S *p_aw_protocol_rx_data_s = AW_NULL;
	PROTOCOL_L2_S *p_protocol_rx_l2_s = AW_NULL;

#ifdef POINTER_CHECK
	if ((p_gui_data_s == AW_NULL) || (p_aw_protocol_rx_data == AW_NULL))
		return AW_FLAG_FAIL;
#endif

	/* Strongly convert array to struct type */
	p_aw_protocol_rx_data_s = (PROTOCOL_DATA_S *)p_aw_protocol_rx_data;
	p_protocol_rx_l2_s = &p_aw_protocol_rx_data_s->protocol_l2_s;
	/* get data length */
	aw_data_len = GET_16_DATA(p_protocol_rx_l2_s->payload_length_h,
					p_protocol_rx_l2_s->payload_length_l);
	/* L3 layer data extraction */
	aw_soc_exchange_buff(p_gui_data_s, p_aw_protocol_rx_data_s,
								aw_data_len);
	p_gui_data_s->module_id = p_protocol_rx_l2_s->module_id;
	p_gui_data_s->event_id = p_protocol_rx_l2_s->event_id;
	p_gui_data_s->err_flag = p_aw_protocol_rx_data_s->
					protocol_l3_data[ERROR_LOCATION_DATA];
	p_gui_data_s->version_num = AW_VERSION_1;
	/* Check data */

	sta_flag = check_protocol_header_data(p_aw_protocol_rx_data_s,
					(ADDRESS_ID_E)p_gui_data_s->src_adr);

	return sta_flag;
}

#endif /* AW_PROTOCOL_MCU */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* SOC_PROTOCOL_VALID */
