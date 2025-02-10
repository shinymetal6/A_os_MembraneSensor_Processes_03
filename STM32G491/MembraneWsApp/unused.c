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
 * unused.c
 *
 *  Created on: Feb 10, 2025
 *      Author: fil
 */
#include "main.h"

#include "../Common/A_os_includes.h"


#ifdef	MEMBRANE_WS_2412171_00
#include "../MembraneWsApp/membrane_includes.h"

typedef enum
{
    GPIO_MODER_INPUT = 0b00,
    GPIO_MODER_OUTPUT = 0b01,
    GPIO_MODER_ALTERNATE = 0b10,
    GPIO_MODER_ANALOG   = 0b11,
}GPIO_MODER;

void GPIO_SetMODER(GPIO_TypeDef *gpio, int pin, GPIO_MODER mode)
{
    uint32_t reg = gpio -> MODER;

    reg &= ~(0b11 << (pin * 2));
    reg |= (mode & 0b11) << (pin * 2);

    gpio -> MODER = reg;
}
#endif

