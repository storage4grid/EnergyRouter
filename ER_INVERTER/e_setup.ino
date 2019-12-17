/* --------------------------------------------------------------------------------------
 * SETUP
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
 * Description: This routine is automatically called at powerup/reset.
 * 
-----------------------------------------------------------------------------------------**/
void setup() {
  
  //set resolutions for analogue ports
  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)
  analogReadResolution(12);   // set the analog input resolution to 12 bit

  //set serial comunication with bitrate = 9600
  Serial.begin(9600);

  //declare digital pins
  pinMode(Error_IGBT1, INPUT);
  pinMode(Error_IGBT2, INPUT);
  pinMode(LED_, OUTPUT);
  pinMode(INV_RELAY, OUTPUT);

  pinMode(13, OUTPUT);//led da placa
  digitalWrite(13, LOW);

  pinMode(SYNC_PIN, INPUT_PULLUP);
  
  //turn off Relays and led
  TURN_OFF_INV_RELAY;

  //vector initialization
  for(int n = 0; n < array_size2; n++)
    V_ARRAY[n] = 0;

  //setup PWM pins
  //2 pwm pins are activated, with sync functionality and 15kHz
  PWM_MAX = setup_pwm(2 , 1, 15000);
  PWM_SUP = DC_max * PWM_MAX;
  PWM_INF = DC_min * PWM_MAX;

  // define an interrupt for input errors
  attachInterrupt(digitalPinToInterrupt(Error_IGBT1), Error_Treatment_INV, RISING);  // Interrupt attached to the pin Error_IGBT1
  attachInterrupt(digitalPinToInterrupt(Error_IGBT2), Error_Treatment_INV, RISING);  // Interrupt attached to the pin Error_IGBT1

  commSetup(); //communications

  TURN_ON_LED;//is ready
}
