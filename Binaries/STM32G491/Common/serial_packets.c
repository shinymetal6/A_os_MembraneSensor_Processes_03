/* 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Project : A_os_MembraneSensor_Processes
*/
/*
 * serial_packets.c
 *
 *  Created on: Sep 12, 2024
 *      Author: fil
 */
#include "main.h"

#include "A_os_includes.h"

#ifdef MEMBRANE_TEMP_2412171_00
#include "../T_App/membrane_includes.h"
#endif
#ifdef MEMBRANE_WS_2412171_00
#include "../WS_App/membrane_includes.h"
#endif

extern	MembraneInfo_TypeDef	MembraneInfo;
extern	MembraneParameters_TypeDef	MembraneParameters;
extern	uint8_t	reprog_data_area[FLASHRAM_SIZE];
extern	uint32_t	uart1_driver_handle;

uint8_t packet_process_commands(void)
{
uint8_t	ret_val = 0;

	if ( MembraneSystem.sensor_rxstate == SENSORS_WAIT_INITIATOR_CHAR)
	{
		if(MembraneSystem.sensor_rxchar == SENSORS_INITIATOR_CHAR)
		{
			bzero(MembraneSystem.sensor_rxbuf,SENSORS_RX_LEN266);
			MembraneSystem.sensor_rxbuf[SENSORS_INITIATOR] = SENSORS_INITIATOR_CHAR;
			MembraneSystem.sensor_rxstate = SENSORS_DATA_PHASE;
			MembraneSystem.sensor_total_rxcount = SENSORS_RX_CMDLEN4;
			MembraneSystem.sensor_rxindex = 1;
		}
	}
	else
	{
		MembraneSystem.sensor_rxbuf[MembraneSystem.sensor_rxindex] = MembraneSystem.sensor_rxchar;
		if ( MembraneSystem.sensor_rxindex == SENSORS_ADDRESS)
		{
			if (MembraneSystem.sensor_rxbuf[SENSORS_ADDRESS] != MembraneInfo.board_address)
				MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;
		}
		else
		{
			if ( MembraneSystem.sensor_rxbuf[SENSORS_CMD] == DOWNLOAD_COMMAND)
			{
				if (( MembraneSystem.sensor_rxindex == SENSORS_RX_LEN266-1) && ( MembraneSystem.sensor_rxbuf[SENSORS_RX_LEN266-1] == SENSORS_TERMINATOR_CHAR))
				{
					MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;
					ret_val = MembraneSystem.sensor_rxbuf[SENSORS_CMD];
				}
			}
			else
			{
				if(MembraneSystem.sensor_rxbuf[SENSORS_ADDRESS+1] == SENSORS_TERMINATOR_CHAR)
				{
					MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;
					ret_val = MembraneSystem.sensor_rxbuf[SENSORS_CMD];
				}
			}
		}
		MembraneSystem.sensor_rxindex ++;
	}
	if ( MembraneSystem.sensor_rxindex >= SENSORS_RX_LEN266)
		MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;
	if ( ret_val )
		MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;
	return ret_val;
}

uint8_t update_packet_process(void)
{
uint32_t	i;

	if ((MembraneSystem.sensors_status & SENSORS_FLASHMODE ) == SENSORS_FLASHMODE )
	{
		MembraneSystem.flash_pkt_crc = do_crc((uint32_t *)&MembraneSystem.sensor_rxbuf[SENSORS_UPDATE_DATA],SENSORS_UPDATE_PAYLOAD+4);
		if ( MembraneSystem.flash_pkt_crc )
		{
			MembraneSystem.flash_download_crc_error++;
			return PKT_CRC_ERROR;
		}
		if (MembraneSystem.flash_pktcntr != MembraneSystem.sensor_rxbuf[SENSORS_UPDATE_PKTCNT])
		{
			MembraneSystem.flash_download_sequence_error++;
			return PKT_SEQ_ERROR;
		}
		for(i=0;i<SENSORS_UPDATE_PAYLOAD;i++)
			MembraneSystem.flash_address[i+(MembraneSystem.flash_pktcntr*SENSORS_UPDATE_PAYLOAD)] = MembraneSystem.sensor_rxbuf[i+SENSORS_UPDATE_DATA];
		MembraneSystem.last_flash_pktcntr_ok = MembraneSystem.flash_pktcntr;
		MembraneSystem.flash_pktcntr++;

		MembraneSystem.flash_counter+=SENSORS_UPDATE_PAYLOAD;
		if ( MembraneSystem.flash_counter >= MembraneSystem.flash_datalen)
		{
			MembraneSystem.sensors_status &= ~SENSORS_FLASHMODE;
			MembraneSystem.flash_flags |= FLASH_READY2FLASH;
			MembraneSystem.cumulative_flash_download_crc_error += MembraneSystem.flash_download_crc_error;
			MembraneSystem.cumulative_flash_download_sequence_error += MembraneSystem.flash_download_sequence_error;
			MembraneSystem.flash_pktcntr = 0;
		}
		return PKT_OK;
	}
	return PKT_NOK_ERROR;
}


