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
 * 18/01/2018 v1.00 First development version.
 * 03/03/2018 v2.00 Functions START_PWM and STOP_PWM were added.
 * 15/09/2018 v2.10 When function STOP_PWM is called, the dutycycle is also set to zero.
-----------------------------------------------------------------------------------------**/
#include <Arduino.h>
#include "h_pwm_due_lib.h"
//{ PIOC, PIO_PC21B_PWML4,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH4,     NOT_ON_TIMER }
/*
Each PIO controller controls up to 32 lines. Each line can be assigned to one of two peripheral functions, A or B.
The multiplexing tables in the following pages define how the I/O lines of the peripherals A and B are multiplexed
on the PIO controllers
{
  Pio* pPort ;						PIO controllers (PIOA, PIOB, PIOC, PIOD, PIOE, and PIOF)
  uint32_t ulPin ;   				PWMLx
  uint32_t uhPin ;   				PWMHx
  uint32_t uPeripheralId ;
  EPioType uPinType ;   			Peripheral functions (PIO_PERIPH_A or PIO_PERIPH_B)
  uint32_t uPinConfiguration ;
  uint32_t uPinAttribute ;
  EPWMChannel uPWMChannel ;
;
} PinDescription_Due_Lib ;*/

/*
 * Pins descriptions
 */
extern const PinDescription_Due_Lib pwm_PinDescription[]=
{
	//   0 .. 4 - PWM channels
  // channel 0
  { PIOC, PIO_PC2B_PWML0, PIO_PC3B_PWMH0, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_PWM, PWM_CH0}, // PIN 34 
  // channel 1
  { PIOC, PIO_PC4B_PWML1, PIO_PC5B_PWMH1, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_PWM, PWM_CH1}, // PIN 36
  // channel 2
  { PIOC, PIO_PC6B_PWML2, PIO_PC7B_PWMH2, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_PWM, PWM_CH2}, // PIN 38
  // channel 3
  { PIOC, PIO_PC8B_PWML3, PIO_PC9B_PWMH3, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_PWM, PWM_CH3}, // PIN 40
  
	
  // END
  { NULL, 0, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NOT_ON_PWM}
} ;

static int PWM_MAX_DUTY = 1400;
static int PWM_FREQ = 15000;

static uint8_t PWM_Activated = 0;
static uint8_t PWM_Sync = 0;
static uint8_t PWM_Actualized[] = {0, 0, 0, 0};

bool complementary[] = {true, true, true, true};
bool PWM_Enabled[] = {false, false, false, false};
/**
 * \brief Configures PWM pins with the given parameters.
 *
 * \param pwm2activate           Numbers of pwm channels to be activated (min = 1 ; max = 4). 
 * \param pwm_sync               Number of pwm channels to be syncronous
 * \param requestMode              PDC transfer request mode.
 * \param requestComparisonSelect  PDC transfer request comparison selection.
 */
unsigned int setup_pwm(unsigned int pwm2activate, unsigned int pwm_sync, unsigned int frequency){
	
	if(pwm2activate <= 4 && pwm2activate > 0 && pwm_sync <= 3 && pwm_sync >= 0 && frequency > 0){
		
		PWM_Activated = pwm2activate;
		PWM_Sync = pwm_sync;
		PWM_FREQ = frequency;
		PWM_MAX_DUTY = 84E6/(PWM_FREQ*PWM_COUNTER*PWM_FREQ_MULTIPLIER);
		
		
//disable all PWM channels
		PWMC_DisableChannel(PWM_INTERFACE, pwm_PinDescription[0].uPWMChannel);
		PWMC_DisableChannel(PWM_INTERFACE, pwm_PinDescription[1].uPWMChannel);
		PWMC_DisableChannel(PWM_INTERFACE, pwm_PinDescription[2].uPWMChannel);
		PWMC_DisableChannel(PWM_INTERFACE, pwm_PinDescription[3].uPWMChannel);
//configure the clock for PWM pins	
		pmc_enable_periph_clk(PWM_INTERFACE_ID); //enable the PWM clock in the Power Management Controller (PMC) before using the PWM
		PWMC_ConfigureClocks(PWM_FREQ * PWM_MAX_DUTY * PWM_FREQ_MULTIPLIER, 0, VARIANT_MCK); //Master Clock (MCK) is the clock of the peripheral bus to which the PWM is connected 

		for ( uint32_t ch = 0; ch < pwm2activate; ch++ ) {
      
//configure PIO for the PWM channel (e.g. channel 0 in pin 34  and pin 35)**********************************************************************************
      //PIO configures the PWM pins as output initially, with its values set to LOW. PWM pins will be configurated as PWM ports when the funcion PWM_START is called in order
      //to mantain the PWM pins at low level and not in high impedance.
			//PIO_Configure(pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].uPinType, pwm_PinDescription[ch].ulPin, pwm_PinDescription[ch].uPinConfiguration);
      //PIO_Configure(pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].uPinType, pwm_PinDescription[ch].uhPin, pwm_PinDescription[ch].uPinConfiguration);
      PIO_SetOutput( pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].ulPin, LOW, 0, PIO_PULLUP );
      PIO_SetOutput( pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].uhPin, LOW, 0, PIO_PULLUP );
	  
//Configure the PWM channels
			/*PWMC_ConfigureChannelExt( Pwm* pPwm, uint32_t ul_channel, uint32_t prescaler, uint32_t alignment, uint32_t polarity,
                                      uint32_t countEventSelect, uint32_t DTEnable, uint32_t DTHInverte, uint32_t DTLInverte ) ; 
			PWMC_ConfigureChannelExt(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
			PWMC_SetPeriod( Pwm* pPwm, uint32_t ul_channel, uint16_t period ) ;
			PWMC_SetDutyCycle( Pwm* pPwm, uint32_t ul_channel, uint16_t duty ) ;
			PWMC_SetDeadTime( Pwm* pPwm, uint32_t ul_channel, uint16_t timeH, uint16_t timeL ) ;*/
	
			PWMC_ConfigureChannelExt(PWM_INTERFACE, pwm_PinDescription[ch].uPWMChannel, PWM_CMR_CPRE_CLKA, PWM__CENTRE_ALIGNED, 0, 0, PWM_DTE_DISABLED, 0, 0);
			PWMC_SetPeriod(PWM_INTERFACE, pwm_PinDescription[ch].uPWMChannel, PWM_MAX_DUTY);
			PWMC_SetDutyCycle(PWM_INTERFACE, pwm_PinDescription[ch].uPWMChannel, 0);
			PWMC_SetDeadTime(PWM_INTERFACE, pwm_PinDescription[ch].uPWMChannel, 0, 0) ; //in nanoseconds PWM_DEADTIME	  
		}
	
		uint32_t sync_ch;
		switch(pwm_sync) {
			case 1  :
				sync_ch = 0x3; //0011
				break; //optional
			case 2  :
				sync_ch = 0x7; //0111
				break; //optional
			case 3  :
				sync_ch = 0xF; //1111 all channels are sync
				break; //optional
			// you can have any number of case statements.
			default :
				sync_ch = 0x0;
		}	
		if(sync_ch != 0) {
		//PWMC_ConfigureSyncChannel( Pwm* pPwm, uint32_t ul_channels, uint32_t updateMode, uint32_t requestMode, uint32_t requestComparisonSelect ) ;
			PWMC_ConfigureSyncChannel(PWM_INTERFACE, sync_ch, 0, 0,0); //make channels 0, 1, 2, 3 be synchronous
			PWMC_SetSyncChannelUpdatePeriod(PWM_INTERFACE, 1);
		}
	
//		for ( uint32_t ch3 = 0; ch3 < pwm2activate; ch3++ ) {
//			//Enable the PWM channels
//			PWMC_EnableChannel(PWM_INTERFACE, pwm_PinDescription[ch3].uPWMChannel);
//		}
	
	}

	//PWM_Enabled = 1;
	return PWM_MAX_DUTY;
}

/**
 * \brief Enable PWM channels.
 *
 * \param non 
 * \param non
 */
void disable_comp(unsigned int pwm_port){
  if(pwm_port >= 1 && pwm_port <= 4 && complementary[pwm_port-1] == true){
  complementary[pwm_port-1] = false;
  PWMC_DisableChannel(PWM_INTERFACE, pwm_PinDescription[pwm_port-1].uPWMChannel);
      //PIO_SetOutput( pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].ulPin, LOW, 0, PIO_PULLUP );
      PIO_SetOutput( pwm_PinDescription[pwm_port-1].pPort, pwm_PinDescription[pwm_port-1].uhPin, LOW, 0, PIO_PULLUP );      
  PWMC_EnableChannel(PWM_INTERFACE, pwm_PinDescription[pwm_port-1].uPWMChannel);
}
}
/**
 * \brief Enable PWM channels.
 *
 * \param non 
 * \param non
 */

void start_pwm(void){
   for ( uint32_t ch = 0; ch < PWM_Activated; ch++ ) {
      if(PWM_Enabled[ch] == false){
      //configure PIO for the PWM channel (e.g. channel 0 in pin 34  and pin 35)
      PIO_Configure( pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].uPinType, pwm_PinDescription[ch].ulPin, pwm_PinDescription[ch].uPinConfiguration);
      if(complementary[ch])
        PIO_Configure( pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].uPinType, pwm_PinDescription[ch].uhPin, pwm_PinDescription[ch].uPinConfiguration);
      //Enable the PWM channels      
      PWMC_EnableChannel(PWM_INTERFACE, pwm_PinDescription[ch].uPWMChannel);
    
      PWM_Enabled[ch] = true;
      }
   }
}

void start_pwm(uint32_t ch){
  if(ch >= 1 && ch <= 4 && PWM_Enabled[ch-1] == false){
   // Serial.print("start pwm ");
   // Serial.println(ch);
      //configure PIO for the PWM channel (e.g. channel 0 in pin 34  and pin 35)
      PIO_Configure( pwm_PinDescription[ch-1].pPort, pwm_PinDescription[ch-1].uPinType, pwm_PinDescription[ch-1].ulPin, pwm_PinDescription[ch-1].uPinConfiguration);
      if(complementary[ch-1])
        PIO_Configure( pwm_PinDescription[ch-1].pPort, pwm_PinDescription[ch-1].uPinType, pwm_PinDescription[ch-1].uhPin, pwm_PinDescription[ch-1].uPinConfiguration);
      //Enable the PWM channels      
      PWMC_EnableChannel(PWM_INTERFACE, pwm_PinDescription[ch-1].uPWMChannel);
    
    PWM_Enabled[ch-1] = true;
  }
}

void stop_pwm(void){
   for ( uint32_t ch = 0; ch < PWM_Activated; ch++ ) {
      if(PWM_Enabled[ch] == true){
	  PWMC_SetDutyCycle(PWM_INTERFACE, ch-1, 0);  
      //Disable the PWM channels
      PWMC_DisableChannel(PWM_INTERFACE, pwm_PinDescription[ch].uPWMChannel);
      //Configure the PWM pin to digital output and value LOW
      //Ex.: PIO_SetOutput(PIOC, PIO_PC20, LOW, 0, PIO_PULLUP);
      PIO_SetOutput( pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].ulPin, LOW, 0, PIO_PULLUP );
      if(complementary[ch])      
        PIO_SetOutput( pwm_PinDescription[ch].pPort, pwm_PinDescription[ch].uhPin, LOW, 0, PIO_PULLUP );
      
      PWM_Enabled[ch] = false;
      }
  }
}

void stop_pwm(uint32_t ch){
  if(ch >= 1 && ch <= 4 && PWM_Enabled[ch-1] == true){
	  PWMC_SetDutyCycle(PWM_INTERFACE, ch-1, 0);
      //Disable the PWM channels
      PWMC_DisableChannel(PWM_INTERFACE, pwm_PinDescription[ch-1].uPWMChannel);
      //Configure the PWM pin to digital output and value LOW
      //Ex.: PIO_SetOutput(PIOC, PIO_PC20, LOW, 0, PIO_PULLUP);
      PIO_SetOutput( pwm_PinDescription[ch-1].pPort, pwm_PinDescription[ch-1].ulPin, LOW, 0, PIO_PULLUP );
      if(complementary[ch-1])      
        PIO_SetOutput( pwm_PinDescription[ch-1].pPort, pwm_PinDescription[ch-1].uhPin, LOW, 0, PIO_PULLUP );
      
      PWM_Enabled[ch-1] = false;
  }
}


/**
 * \brief Update the duty-cycle of pwm channel.
 *
 * \param pwm_port           			PWM port (min = 1 ; max = 4). 
 * \param DCvalue               Duty cycle value
 */
void updatePWM(unsigned int pwm_port, unsigned int dc_value){
	
	if(PWM_Enabled[pwm_port-1] == true) {
		if( pwm_port > 0 && pwm_port <= PWM_Activated  && dc_value >= 0 && dc_value <= PWM_MAX_DUTY  && PWM_Sync == 0) {
			PWMC_SetDutyCycle(PWM_INTERFACE, pwm_port-1, dc_value);
		}
		else {
			PWMC_SetDutyCycle(PWM_INTERFACE, pwm_port-1, 0);
		}
	}
}

void update_SyncPWM(unsigned int dc_1, unsigned int dc_2){
	
	if(PWM_Enabled[0] == true && PWM_Enabled[1] == true && PWM_Sync == 1) {
		if( (dc_1 < 0 && dc_1 > PWM_MAX_DUTY) || (dc_2 < 0 && dc_2 > PWM_MAX_DUTY)) {
			PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0);
			PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0);
			PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE);
		} 
		else {
			PWMC_SetDutyCycle(PWM_INTERFACE, 0, dc_1);
			PWMC_SetDutyCycle(PWM_INTERFACE, 1, dc_2);
			PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE);
		}
	
	}
}
void update_SyncPWM(unsigned int dc_1, unsigned int dc_2, unsigned int dc_3){
	
	if(PWM_Enabled[0] == true && PWM_Enabled[1] == true && PWM_Enabled[2] == true && PWM_Sync == 2) {
		
		if( (dc_1 < 0 && dc_1 > PWM_MAX_DUTY) || (dc_2 < 0 && dc_2 > PWM_MAX_DUTY) || (dc_3 < 0 && dc_3 > PWM_MAX_DUTY)) {
			PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0);
			PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0);
			PWMC_SetDutyCycle(PWM_INTERFACE, 2, 0);
			PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE);
		} 
		else {
			PWMC_SetDutyCycle(PWM_INTERFACE, 0, dc_1);
			PWMC_SetDutyCycle(PWM_INTERFACE, 1, dc_2);
			PWMC_SetDutyCycle(PWM_INTERFACE, 2, dc_3);
			PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE);
		}
	
	}
}

void update_SyncPWM(unsigned int dc_1, unsigned int dc_2, unsigned int dc_3, unsigned int dc_4){
	
	if(PWM_Enabled[0] == true && PWM_Enabled[1] == true && PWM_Enabled[2] == true && PWM_Enabled[3] == true && PWM_Sync == 3) {
		
		if( (dc_1 < 0 && dc_1 > PWM_MAX_DUTY) || (dc_2 < 0 && dc_2 > PWM_MAX_DUTY) || (dc_3 < 0 && dc_3 > PWM_MAX_DUTY) || (dc_4 < 0 && dc_4 > PWM_MAX_DUTY)) {
			PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0);
			PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0);
			PWMC_SetDutyCycle(PWM_INTERFACE, 2, 0);
			PWMC_SetDutyCycle(PWM_INTERFACE, 3, 0);
			PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE);
		} 
		else {
			PWMC_SetDutyCycle(PWM_INTERFACE, 0, dc_1);
			PWMC_SetDutyCycle(PWM_INTERFACE, 1, dc_2);
			PWMC_SetDutyCycle(PWM_INTERFACE, 2, dc_3);
			PWMC_SetDutyCycle(PWM_INTERFACE, 3, dc_4);
			PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE);
		}
	
	}
}
