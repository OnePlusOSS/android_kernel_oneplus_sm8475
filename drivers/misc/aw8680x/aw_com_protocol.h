/*
* Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
* All rights reserved.
* Description: Communication protocol related header file
*/
#ifndef __AW_COM_PROTOCOL_H
#define __AW_COM_PROTOCOL_H
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define GET_HIGH_4BIT(data)		(((data) & 0xf0)>>4)
#define GET_LOW_4BIT(data)		((data) & 0x0f)

#ifdef AW_PROTOCOL_MCU

#define UI_LENGTH_LEN	(2U)
#define NO_RESPOND_LEN	(3U)
#define UI_ADDR_LEN		(4U)
#define UI_L1_L2_LEN	(9U)

AW_U8 aw_soc_protocol_pack(GUI_TO_SOC_S *p_gui_data_s,
						AW_U8 *p_aw_protocol_tx_data);
AW_CHECK_FLAG_E aw_soc_protoco_unpack(GUI_TO_SOC_S *p_gui_data_s,
						AW_U8 *p_aw_protocol_rx_data);

#endif /* AW_PROTOCOL_MCU */

AW_U8 check_sum(AW_U8 *buf, AW_U16 len);
AW_CHECK_FLAG_E check_protocol_header_data(PROTOCOL_DATA_S *
				p_aw_protocol_rx_data_s, ADDRESS_ID_E aw_id_e);
AW_U8 aw_set_protocol_header_data(PROTOCOL_DATA_S *p_aw_protocol_tx_data_s,
							AW_U16 data_len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
