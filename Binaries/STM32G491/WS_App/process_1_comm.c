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
 * process_1_comm.c
 *
 *  Created on: May 20, 2024
 *      Author: fil
 */


/*
 * 	sprintf((char *)MembraneSystem.update_line,"<U0 datalen %d U>",(int )len );
 *
 */
#include "main.h"

#include "../../STM32G491/Common/A_os_includes.h"


#ifdef	MEMBRANE_WS_2412171_00
#include "membrane_includes.h"

#define	SENSVERSION	"Mws 1.3.0c"

extern	MembraneInfo_TypeDef		MembraneFlashInfo;
extern	MembraneInfo_TypeDef		MembraneInfo;
extern	MembraneParameters_TypeDef	MembraneFlashParameters;
extern	MembraneParameters_TypeDef	MembraneParameters;

extern	uint8_t						reprog_data_area[FLASHRAM_SIZE];


MembraneSystem_TypeDef				MembraneSystem;
AcqSystem_TypeDef					AcqSystem;

uint8_t 							uart_rx_buffer[SENSORS_RX_LEN266+10];

#define	UART_RX_BUF_SIZE	280
#define	UART_TX_BUF_SIZE	128
uint8_t	uart1_rx_buffer[UART_RX_BUF_SIZE];
uint8_t	uart1_tx_buffer[UART_TX_BUF_SIZE];

UART_Drv_TypeDef Uart1_Drv =
{
	.data = &MembraneSystem.sensor_rxchar,
	.rx_max_len = 1,
	.uart = &huart1,
	.wakeup_id = WAKEUP_FROM_UART1_IRQ,
	.timeout = 10,
	.flags = UART_USES_DMA_TX | UART_WAKEUP_ON_RXCHAR,
};

uint32_t	uart1_driver_handle;

uint16_t	adc_data[3];

ADC_Drv_TypeDef	ADC_Drv =
{
		.adc = &hadc1,
		.adc_buffer = adc_data,
		.num_channels = 3,
		.adc_timer = &htim6,
		.flags = ADC_FLAGS_FULL_WAKEUP | ADC_FLAGS_CALIBRATE,
		.wakeup_id = WAKEUP_FROM_ADC1_IRQ,
};
uint32_t		adc_driver_handle;

uint32_t							sizeMembraneFlashInfo,sizeMembraneParameters;

//#define	 DEBUG_ALGO	1
#ifdef DEBUG_ALGO
AcqSystem_TypeDef		AcqSystem;

uint8_t	debug_algo_tim = 0;
uint32_t	conductivity,adc_raw_value;

#endif

void compile_version_and_send(void)
{
uint16_t	len;
	bzero(uart1_tx_buffer,UART_TX_BUF_SIZE);
	sprintf((char *)&uart1_tx_buffer[1],"<J00 %s>",SENSVERSION);
	len = strlen((char *)&uart1_tx_buffer[1]);
	uart1_tx_buffer[3] = MembraneInfo.board_address;
	uart1_tx_buffer[4] = MembraneInfo.board_type;
	uart_send(uart1_driver_handle, uart1_tx_buffer,len+1);
}

void compile_flash_reply_and_send(uint8_t command,uint8_t result)
{
	uart1_tx_buffer[0] = 0;
	uart1_tx_buffer[1] = '<';
	uart1_tx_buffer[2] = command;
	uart1_tx_buffer[3] = MembraneInfo.board_address;
	uart1_tx_buffer[4] = MembraneInfo.board_type;
	uart1_tx_buffer[5] = result;
	uart1_tx_buffer[6] = MembraneSystem.last_flash_pktcntr_ok;
	uart1_tx_buffer[7] = '>';
	uart1_tx_buffer[8] = 0;
	uart_send(uart1_driver_handle, uart1_tx_buffer,9);
}

void compile_data_and_send(void)
{
	uart1_tx_buffer[0] = 0;
	uart1_tx_buffer[SENSORS_INITIATOR+1] = '<';
	uart1_tx_buffer[SENSORS_CMD+1] = SENSORS_GET_DATA;
	uart1_tx_buffer[SENSORS_ADDRESS+1] = MembraneInfo.board_address;
	uart1_tx_buffer[SENSORS_TYPE+1] = MembraneInfo.board_type;
	uart1_tx_buffer[SENSORS_TYPE+2] = AcqSystem.internal_scale_factor;
	uart1_tx_buffer[SENSORS_TYPE+3] = AcqSystem.conductivity_value>>8;
	uart1_tx_buffer[SENSORS_TYPE+4] = AcqSystem.conductivity_value & 0xff;
	uart1_tx_buffer[SENSORS_TYPE+5] = AcqSystem.temperature_data>>8;
	uart1_tx_buffer[SENSORS_TYPE+6] = AcqSystem.temperature_data& 0xff;
	uart1_tx_buffer[SENSORS_TYPE+7] = '>';
	uart1_tx_buffer[SENSORS_TYPE+8] = 0;
	uart_send(uart1_driver_handle, uart1_tx_buffer,12);
}

void process_1_comm(uint32_t process_id)
{
uint32_t	wakeup,flags;
uint8_t		packet_process_commands_result;
uint8_t		send_version_info=0;

	sizeMembraneFlashInfo = sizeof(MembraneFlashInfo);
	bcopy(&MembraneFlashInfo,&MembraneInfo,sizeMembraneFlashInfo);
	sizeMembraneParameters = sizeof(MembraneFlashParameters);
	bcopy(&MembraneFlashParameters,&MembraneParameters,sizeMembraneParameters);
	uart1_driver_handle = uart_register(&Uart1_Drv);
	uart_start_receive(uart1_driver_handle);

	adc_driver_handle = int_adc_register(&ADC_Drv);
	adc_start(adc_driver_handle);
	algo_init();

	create_timer(TIMER_ID_0,10,TIMERFLAGS_FOREVER | TIMERFLAGS_ENABLED);
	MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;

	while(1)
	{
		wait_event(EVENT_TIMER | EVENT_UART1_IRQ | EVENT_ADC1_IRQ | EVENT_DAC_IRQ);
		get_wakeup_flags(&wakeup,&flags);

		if (( wakeup & WAKEUP_FROM_TIMER) == WAKEUP_FROM_TIMER)
		{
#ifdef DEBUG_ALGO
			debug_algo_tim++;
			if ( debug_algo_tim == 8 )
				algo_start();
			if ( debug_algo_tim > 9 )
				debug_algo_tim = 0;
#endif
			if ( send_version_info )
			{
				send_version_info = 0;
				compile_version_and_send();
				MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;
			}
		}
		if (( wakeup & WAKEUP_FROM_UART1_IRQ) == WAKEUP_FROM_UART1_IRQ)
		{
			if (( flags & WAKEUP_FLAGS_UART_RX) == WAKEUP_FLAGS_UART_RX)
			{
				packet_process_commands_result = packet_process_commands();
				if ( packet_process_commands_result )
				{
					if ( packet_process_commands_result == SENSORS_GET_DATA )
					{
						algo_start();
						MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;
					}

					if ( packet_process_commands_result == SENSORS_GET_VERSION )
						send_version_info=1;

					if ( packet_process_commands_result == DOWNLOAD_PREPARE_COMMAND )
					{
						MembraneSystem.flash_flags = 0;
						MembraneSystem.sensors_status |= SENSORS_FLASHMODE;
						MembraneSystem.flash_datalen = FLASH_MAXLEN;
						MembraneSystem.sensor_rxindex = 0;
						MembraneSystem.sensor_rxstate = SENSORS_WAIT_INITIATOR_CHAR;
						MembraneSystem.flash_pktcntr = 0;
						MembraneSystem.flash_download_crc_error = MembraneSystem.flash_download_sequence_error = MembraneSystem.flash_counter = 0;
						clear_flash_area();
						compile_flash_reply_and_send(DOWNLOAD_PREPARE_COMMAND,'Y');
					}

					if ( packet_process_commands_result == DOWNLOAD_COMMAND )
					{
						compile_flash_reply_and_send(DOWNLOAD_COMMAND,update_packet_process() );
					}
					if ( packet_process_commands_result == WRITE_FLASH_COMMAND )
					{
						do_flash_update(MembraneSystem.flash_address,FLASH_MAXLEN);
					}
				}
			}
		}
		if (( wakeup & WAKEUP_FROM_ADC1_IRQ) == WAKEUP_FROM_ADC1_IRQ)
		{
			if (( AcqSystem.acquisition_status & ACQ_ADC_RUN) == ACQ_ADC_RUN)
			{
				AcqSystem.temperature_state = 0;
				AcqSystem.algo_samples_index &= 0x03;
				AcqSystem.algo_samples[AcqSystem.algo_samples_index] = adc_data[ADC1_OPAMP_INDEX];
				AcqSystem.algo_samples_index++;
				if ( AcqSystem.algo_samples_index > 3)
				{
					AcqSystem.algo_samples_index = 0;
					AcqSystem.adc_in_value = (AcqSystem.algo_samples[2] + AcqSystem.algo_samples[3])/2;
					apply_algo();
#ifndef DEBUG_ALGO
					compile_data_and_send();
#endif
				}
			}
#ifndef DEBUG_ALGO

			else
			{
				switch(AcqSystem.temperature_state)
				{
				case 2 :
					hadc1.Instance->SMPR2 = 0x6180000;
					AcqSystem.temperature_state++;
					break;
				case 8 :
					AcqSystem.vrefint_data	= __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc_data[ADC1_VREFINT_INDEX], LL_ADC_RESOLUTION_12B);
					AcqSystem.tmpr_data 	=__LL_ADC_CALC_TEMPERATURE(AcqSystem.vrefint_data, adc_data[ADC1_TEMPERATURE_INDEX], LL_ADC_RESOLUTION_12B);
					AcqSystem.tmpr_data -= 5;
					AcqSystem.temperature_data = (uint16_t )AcqSystem.tmpr_data;
					hadc1.Instance->SMPR2 = 0;
					AcqSystem.temperature_state++;
					break;
				case 12 :
					AcqSystem.temperature_state--;
				default :
					AcqSystem.temperature_state++;
				}
			}
#endif
		}
	}
}

#endif // #ifdef	MEMBRANE_WS_2412171_00
