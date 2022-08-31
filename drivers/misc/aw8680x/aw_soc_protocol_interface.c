/*
* Copyright © Shanghai Awinic Technology Co., Ltd. 2019 2020 .
* All rights reserved.
* Description: Communication protocol fun related header file
*/
#include "aw_protocol_config.h"
#include "aw_protocol_map.h"
#include "aw_com_protocol.h"
#include "aw_protocol_fun.h"
#include "aw_soc_protocol_interface.h"

/*
 * @version_num: consistent
 * @ui_rd_data_len: Fixed when packaged and sent
 * @device_commu: UI use
 * @device_addr: i2c device addr
 * @dest_adr: Destination address means: to whom?
 * @src_adr: source addres means:form where?
 * @err_flag: Operation status of lower computer
 */
void aw_set_pack_fixed_data(GUI_TO_SOC_S *p_gui_data_s)
{
	p_gui_data_s->version_num = 0x0001;
	p_gui_data_s->ui_rd_data_len = 0x0000;
	p_gui_data_s->device_commu = 0x01;
	p_gui_data_s->device_addr = 0x5c;
	p_gui_data_s->dest_adr = 0x08;
	p_gui_data_s->src_adr = 0x01;
	p_gui_data_s->err_flag = 0x00;
	p_gui_data_s->reserved0 = 0x00;
	p_gui_data_s->reserved1 = 0x00;
}

AW_U8 aw_soc_protocol_pack_interface(GUI_TO_SOC_S *p_gui_data_s,
				     AW_U8 *p_aw_protocol_tx_data)
{
	aw_set_pack_fixed_data(p_gui_data_s);

	return aw_soc_protocol_pack(p_gui_data_s, p_aw_protocol_tx_data);
}

AW_U8 aw_soc_protocol_unpack_interface(GUI_TO_SOC_S *p_gui_data_s,
				       AW_U8 *p_aw_protocol_rx_data)
{
	return aw_soc_protoco_unpack(p_gui_data_s, p_aw_protocol_rx_data);
}
