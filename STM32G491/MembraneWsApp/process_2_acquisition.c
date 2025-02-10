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
 * process_2_acquisition.c
 *
 *  Created on: May 20, 2024
 *      Author: fil
 */

#include "main.h"

#include "../Common/A_os_includes.h"

#ifdef	MEMBRANE_WS_2412171_00
#include "membrane_includes.h"


extern	AcqSystem_TypeDef		AcqSystem;
/* adc1 in1 vopamp -> byte 0 */
/* adc1 vrefint    -> byte 1 */
/* adc1 temp       -> byte 2 */

extern	 uint16_t	analog_buffer[NUM_ADC1_CHANNELS];
/* adc2 in4 connected to dac -> byte 0 */
/* adc2 in2 connected input  -> byte 1 */
extern	 uint16_t	calibration_buffer[NUM_ADC2_CHANNELS];

uint8_t				from_prc1_mbx[PRC1_MAILBOX_LEN];

//#define TEST_ALGO	1
#ifdef TEST_ALGO
#define	ROUNDTRIP_ALGO_TEST_TIM	50
uint8_t	algo_test_tim = 0;
#endif

void process_2_acquisition(uint32_t process_id)
{
uint32_t	wakeup,flags;

	allocate_hw(HW_ADC1,0);
	allocate_hw(HW_ADC2,0);
	IntAdc_Init(HW_ADC1,(uint32_t *)analog_buffer,NUM_ADC1_CHANNELS);
	IntAdc_Init(HW_ADC2,(uint32_t *)calibration_buffer,NUM_ADC2_CHANNELS);

	algo_init();

	create_timer(TIMER_ID_0,10,TIMERFLAGS_FOREVER | TIMERFLAGS_ENABLED);

	while(1)
	{
		wait_event(EVENT_TIMER | EVENT_ADC1_IRQ | EVENT_MBX);
		get_wakeup_flags(&wakeup,&flags);

  		if (( wakeup & WAKEUP_FROM_MBX) == WAKEUP_FROM_MBX)
		{
#ifndef TEST_ALGO
			if ( mbx_receive(PRC1_MAILBOX_ID,from_prc1_mbx) == PRC1_MAILBOX_MSGLEN)
			{
				if (( from_prc1_mbx[0] == 0xde ) && ( from_prc1_mbx[1] == 0xad )&&( from_prc1_mbx[2] == 0xbe ) && ( from_prc1_mbx[3] == 0xef ))
					algo_run_acquisition();
			}
#endif
		}

		if (( wakeup & WAKEUP_FROM_TIMER) == WAKEUP_FROM_TIMER)
		{
#ifdef TEST_ALGO
  			algo_test_tim++;
  			if ( algo_test_tim >= ROUNDTRIP_ALGO_TEST_TIM )
  			{
				algo_run_acquisition();
				algo_test_tim = 0;
  			}
#endif
			algo_periodic_worker();
		}

		if (( wakeup & WAKEUP_FROM_ADC1_IRQ) == WAKEUP_FROM_ADC1_IRQ)
		{
			algo_acquisition_worker();
		}
	}
}

#endif // #ifdef	MEMBRANE_2412171_00
