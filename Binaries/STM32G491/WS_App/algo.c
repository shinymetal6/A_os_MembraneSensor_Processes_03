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
 * Project : A_os
*/
/*
 * algo.c
 *
 *  Created on: Feb 17, 2025
 *      Author: fil
 */


#include "main.h"

#include "../../STM32G491/Common/A_os_includes.h"


#ifdef	MEMBRANE_WS_2412171_00
#include "membrane_includes.h"

void set_opamp_gain(uint8_t gain)
{
	if ( gain < 7 )
	{
		hopamp1.Instance->CSR &= ~OPAMP_CSR_PGGAIN_Msk;
		hopamp1.Instance->CSR |= ((gain << OPAMP_CSR_PGGAIN_Pos) | OPAMP_CSR_PGGAIN_3);
	}
}

void algo_init(void)
{
	HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_OPAMP_Start(&hopamp1);
	HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	set_opamp_gain(0);
}

void algo_start(void)
{
	AcqSystem.dac_out_value = DAC_OUT_VALUE;
	HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, AcqSystem.dac_out_value);
	AcqSystem.acquisition_status &= ~ACQ_COMPLETE;
	AcqSystem.acquisition_status |= ACQ_ADC_RUN;
	AcqSystem.algo_samples_index = 0;
	AcqSystem.internal_scale_factor = 0;
	HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_SET);
}

void algo_stop(void)
{
	AcqSystem.dac_out_value = 0;
	HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, AcqSystem.dac_out_value);
	AcqSystem.algo_samples_index = 0;
	HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_RESET);
	AcqSystem.acquisition_status &= ~ACQ_ADC_RUN;
	AcqSystem.acquisition_status |= ACQ_COMPLETE;
}

uint8_t apply_algo(void)
{
uint16_t i;
	for(i=0;i<32;i++)
	{
		if ( AcqSystem.adc_in_value < PARAM_THRESHOLD_MIN )
		{
			if ( AcqSystem.internal_scale_factor )
			{
				AcqSystem.internal_scale_factor--;
				AcqSystem.dac_out_value *= 2;
				HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, AcqSystem.dac_out_value);
				return 1;
			}
		}
		else if ( AcqSystem.adc_in_value > PARAM_THRESHOLD_MAX )
		{
			AcqSystem.dac_out_value /= 2;
			AcqSystem.internal_scale_factor++;
			HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, AcqSystem.dac_out_value);
			return 1;
		}
		else
			break;
	}
	AcqSystem.conductivity_value = AcqSystem.adc_in_value;
	algo_stop();
	return 0;
}
#endif // #ifdef	MEMBRANE_WS_2412171_00

