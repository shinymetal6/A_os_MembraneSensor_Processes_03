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
 * algo.c
 *
 *  Created on: Sep 13, 2024
 *      Author: fil
 */

#include "main.h"

#include "../Common/A_os_includes.h"


#ifdef	MEMBRANE_WS_2412171_00
#include "../MembraneWsApp/membrane_includes.h"

extern	DAC_HandleTypeDef hdac1;
extern	OPAMP_HandleTypeDef hopamp1;

extern	MembraneInfo_TypeDef		MembraneFlashInfo;
extern	MembraneInfo_TypeDef		MembraneInfo;
extern	MembraneParameters_TypeDef	MembraneFlashParameters;
extern	MembraneParameters_TypeDef	MembraneParameters;

__attribute__ ((aligned (32)))AcqSystem_TypeDef		AcqSystem;

__attribute__ ((aligned (32))) uint16_t	analog_buffer[NUM_ADC1_CHANNELS];
__attribute__ ((aligned (32))) uint16_t	calibration_buffer[NUM_ADC2_CHANNELS];

__attribute__ ((aligned (32))) uint16_t	acquisition_results[ADC_NUM_ACQUISITION_CYCLES];
__attribute__ ((aligned (32))) uint16_t	direct_acquisition_results[ADC_NUM_ACQUISITION_CYCLES];
__attribute__ ((aligned (32))) uint16_t	zerobuffer_results[ADC_NUM_ACQUISITION_CYCLES];

void algo_init(void)
{
	AcqSystem.dac_out_value = DAC_OUT_VALUE;
	AcqSystem.internal_scale_factor = 0;
	AcqSystem.dac_state_machine = ADC_STATE_IDLE;
	AcqSystem.acquisition_status = ACQ_COMPLETE;
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
}


void algo_run_acquisition(void)
{
	if (( AcqSystem.acquisition_status & ACQ_COMPLETE ) == ACQ_COMPLETE)
	{
		AcqSystem.acquisition_status &= ~ACQ_COMPLETE;
		AcqSystem.acquisition_status |= ACQ_ADC_RUN;
		AcqSystem.dac_state_machine = ADC_STATE_IDLE;
	}
}

void algo_periodic_worker(void)
{
	if (( AcqSystem.acquisition_status & ACQ_ADC_RUN) == ACQ_ADC_RUN)
	{
		switch( AcqSystem.dac_state_machine )
		{
		case	ADC_STATE_IDLE :
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
			IntAdc_Start(HW_ADC1);
			IntAdc_Start(HW_ADC2);
			AcqSystem.adc_index = 0;
			AcqSystem.dac_state_machine = ADC_NOISE_STATE_ACQUISITION;
			break;
		case	ADC_NOISE_STATE_ACQUISITION :
			if (( AcqSystem.acquisition_status & ACQ_NOISE_GET_COMPLETE ) == ACQ_NOISE_GET_COMPLETE)
			{
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_MAX_VALUE);
				AcqSystem.dac_state_machine = ADC_STATE_ACQUISITION;
			}
			break;
		case	ADC_STATE_ACQUISITION :
			if (( AcqSystem.acquisition_status & ACQ_DAC_GEN_COMPLETE ) == ACQ_DAC_GEN_COMPLETE)
			{
				IntAdc_Stop(HW_ADC1);
				IntAdc_Stop(HW_ADC2);
				AcqSystem.dac_state_machine = ADC_STATE_CYCLE_END;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
			}
			break;
		case	ADC_STATE_CYCLE_END :
			AcqSystem.dac_state_machine = ADC_STATE_IDLE;
			AcqSystem.acquisition_status = ACQ_COMPLETE;
			AcqSystem.acquisition_status &= ~(ACQ_ADC_RUN | ACQ_DAC_GEN_COMPLETE | ACQ_NOISE_GET_COMPLETE);
			break;
		}
	}
}

void calc_values(void)
{
uint32_t	i;
	/* finding values */
	AcqSystem.acquisition_raw_value = AcqSystem.noise_raw_value = 0;

	for(i=ADC_NOISE_ACQUISITION_CYCLES+ADC_ACQUISITION_OFFSET;i<ADC_NUM_ACQUISITION_CYCLES;i++)
	{
		AcqSystem.acquisition_raw_value += acquisition_results[i];
	}
	for(i=0;i<ADC_NOISE_ACQUISITION_CYCLES;i++)
	{
		AcqSystem.noise_raw_value += (DAC_MAX_VALUE - acquisition_results[i]);
	}
	AcqSystem.internal_scale_factor = 0;
	AcqSystem.noise_value = AcqSystem.noise_raw_value / ADC_NOISE_ACQUISITION_CYCLES;
	AcqSystem.conductivity_value = AcqSystem.acquisition_raw_value / ADC_NUM_ACQUISITION_CYCLES;
	if ( AcqSystem.conductivity_value > AcqSystem.noise_value )
		AcqSystem.conductivity_value -=  AcqSystem.noise_value;

	AcqSystem.conductivity_value = acquisition_results[ADC_NOISE_ACQUISITION_CYCLES+ADC_ACQUISITION_OFFSET];
	if ( AcqSystem.conductivity_value > AcqSystem.noise_value)
		AcqSystem.conductivity_value -= AcqSystem.noise_value;
	while ( AcqSystem.conductivity_value > 1024)
	{
		if ( AcqSystem.internal_scale_factor < 7 )
		{
			AcqSystem.conductivity_value >>= 1;
			AcqSystem.internal_scale_factor++;
		}
		else
			break;
	}
	AcqSystem.vrefint_data     = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analog_buffer[ADC1_VREFINT_INDEX], LL_ADC_RESOLUTION_12B);
	AcqSystem.temperature_data = __LL_ADC_CALC_TEMPERATURE(AcqSystem.vrefint_data, analog_buffer[ADC1_TEMPERATURE_INDEX], LL_ADC_RESOLUTION_12B);
	AcqSystem.temperature_data -= 5;
	AcqSystem.calibration_value = 0;
}

#define	NEGATE

void algo_acquisition_worker(void)
{
#ifdef NEGATE
	acquisition_results[AcqSystem.adc_index] = DAC_OUT_VALUE -  analog_buffer[ADC1_OPAMP_INDEX];
	direct_acquisition_results[AcqSystem.adc_index] = DAC_OUT_VALUE -  analog_buffer[ADC1_ANALOG_INDEX];
#else
	acquisition_results[AcqSystem.adc_index] = analog_buffer[ADC1_OPAMP_INDEX];
	direct_acquisition_results[AcqSystem.adc_index] = analog_buffer[ADC1_ANALOG_INDEX];
#endif

	if ( AcqSystem.adc_index == ADC_NOISE_ACQUISITION_CYCLES )
	{
		AcqSystem.acquisition_status |= ACQ_NOISE_GET_COMPLETE;
	}
	else if ( AcqSystem.adc_index == ADC_TOTAL_CYCLES )
	{
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		AcqSystem.acquisition_status |= ACQ_ADC_CYCLE_COMPLETE;
		AcqSystem.acquisition_status |= ACQ_DAC_GEN_COMPLETE;
		calc_values();

	}
	AcqSystem.adc_index++;
}

#endif // #ifdef	MEMBRANE_2412171_00

