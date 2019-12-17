/* --------------------------------------------------------------------------------------
 * PWM_DUE_LIB library
 * --------------------------------------------------------------------------------------
 * Copyright (c) 2019 Nuno Vilhena <nuv@uninova.pt>
 *
 * This file is part of ER Inverter.
 *
 * ER Inverter is free software; you can redistribute it and/or
 * modify it under the terms of the GNU GENERAL PUBLIC LICENSE
 * as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public
 * License along with ER Inverter. If not, see: <http://www.gnu.org/licenses/>.
 * --------------------------------------------------------------------------------------
 *
 * Description: This file includes classes and objects for using with Arduino
 * 				DUE's ATMEL ATSAM3X8E micro-controller PWM modules.
 *        PWM channels that can be used: PWMx0 (D34 & D35), PWMx1 (D36 & D37), PWMx2 (D38 & D39), PWMx3 (D40 & D41)
 * 18/01/2018 v1.00 First development version.
 * 03/03/2018 v2.00 Functions START_PWM and STOP_PWM were added.
 * 15/09/2018 v2.10 When function STOP_PWM is called, the dutycycle is also set to zero.
 * 
 * Microcontroller DataSheet: <atmel-11057-32-bit-cortex-m3-microcontroller-sam3x-sam3a_datasheet.pdf>
 *
 * Other similar projects that can help you:
 * <https://github.com/collin80/DueMotorCtrl>
 * <https://github.com/antodom/pwm_lib>
 * --------------------------------------------------------------------------------------
 * Main Directory:	<C:*\ArduinoData\packages\arduino\hardware\sam\1.6.11>
 * Variants:		<C:*\ArduinoData\packages\arduino\hardware\sam\1.6.11\variants\arduino_due_x>
 * Components: 		<C:*\ArduinoData\packages\arduino\hardware\sam\1.6.11\system\CMSIS\Device\ATMEL\sam3xa>
 * Libraries:		<C:*\ArduinoData\packages\arduino\hardware\sam\1.6.11\system\libsam>
 * 
-----------------------------------------------------------------------------------------**/

#ifndef PWM_DUE_LIB_H
#define PWM_DUE_LIB_H
#include <Arduino.h>

//The two values PWM_FREQ and MAX_PWM_DUTY should form an even divisor to the 84Mhz main clock
//The clock used can be from 1x divisor to 255 but will be integer increments
//The equation is 84Mhz / (Freq * DutyCycle)
//So, with 15000 Hz PWM we get 84 MHz / 15000 Hz = 5600.
//The max duty being 511 means 5600/511 turns into 10.96.
//The register can take a value of 10 or 11 so this works out, but the frequency won't be exactly 15 kHz.



#define PWM_FREQ2			15000
#define MAX_PWM_DUTY2		1400 //max value duty cycle can take 9 bits
#define PWM_CLOCK			(PWM_FREQ2 * MAX_PWM_DUTY2) //number of PWM ticks per second - autocalculated - don't modify this line
#define PWM_PICO_PER_TICK	(1000000000ul / (PWM_CLOCK / 1000ul)) //Number of picoSeconds per PWM clock tick - trust me, it's relevant

//target dead time in nano seconds. So, 1000 is a micro second. You will need to tune this to the IGBT hardware.
#define PWM_TARGET_DEADTIME	1400
//autocalculated from constant parameters. Don't change this, change the input parameters above
#define PWM_DEADTIME		(((PWM_TARGET_DEADTIME * 1000ul) / PWM_PICO_PER_TICK) + 1)

//If the PWM pulse would be shorter than this many nano seconds then suppress it.
#define PWM_BUFFER_NANOS	300
//this is an autocalculated value based on a bunch of the above constants. Do not change this equation.
#define PWM_BUFFER			(((PWM_BUFFER_NANOS * 500ul) / PWM_PICO_PER_TICK) + (PWM_DEADTIME * 2))


#define PWM_DC_RES (0x1u << 12); //12 bits

#define PWM__CENTRE_ALIGNED	PWM_CMR_CALG		// centre aligned ; 0 - left aligned
#define PWM__LEFT_ALIGNED	0		// centre aligned
#define PWM_DTE_ENABLED PWM_CMR_DTE //dead time enabled
#define PWM_DTE_DISABLED 0 //dead time enabled
#define PWM_COUNTER 2
#define PWM_FREQ_MULTIPLIER 2	//if the aligned is defined do cetre, this value should be 2

typedef struct _PinDescription_Due_Lib
{
  Pio* pPort ;
  uint32_t ulPin ;
  uint32_t uhPin ;
  uint32_t uPeripheralId ;
  EPioType uPinType ;
  uint32_t uPinConfiguration ;
  uint32_t uPinAttribute ;
  EPWMChannel uPWMChannel ;
} PinDescription_Due_Lib ;



/* Pins table to be instanciated into variant.cpp */
extern const PinDescription_Due_Lib pwm_PinDescription[] ;



unsigned int setup_pwm(unsigned int pwm2activate, unsigned int pwm_sync, unsigned int frequency);
void disable_comp(unsigned int pwm_port);
void start_pwm(void);
void start_pwm(uint32_t ch);
void stop_pwm(void);
void stop_pwm(uint32_t ch);

void updatePWM(unsigned int pwm_port, unsigned int dc_value);
void update_SyncPWM(unsigned int dc_1, unsigned int dc_2);
void update_SyncPWM(unsigned int dc_1, unsigned int dc_2, unsigned int dc_3);
void update_SyncPWM(unsigned int dc_1, unsigned int dc_2, unsigned int dc_3, unsigned int dc_4);

#endif
