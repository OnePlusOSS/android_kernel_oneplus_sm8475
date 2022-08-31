/*
* Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2019 .
* All rights reserved.
* Description: Communication protocol related header file
*/
#ifndef __AW_SOC_PROTOCOL_INTERFACE_H
#define __AW_SOC_PROTOCOL_INTERFACE_H
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

AW_U8 aw_soc_protocol_pack_interface(GUI_TO_SOC_S *p_gui_data_s,
				     AW_U8 *p_aw_protocol_tx_data);
AW_U8 aw_soc_protocol_unpack_interface(GUI_TO_SOC_S *p_gui_data_s,
				       AW_U8 *p_aw_protocol_rx_data);

#endif
