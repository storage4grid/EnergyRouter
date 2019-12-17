/* --------------------------------------------------------------------------------------
 * Communication variables
 * --------------------------------------------------------------------------------------
 * Copyright (c) 2019 Nuno Vilhena <nuv@uninova.pt>
 *
 * This file is part of ER Inverter.
 *
 * ER Inverteris free software; you can redistribute it and/or
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
 * Description: This file includes the communication variables
  * 
-----------------------------------------------------------------------------------------**/

//unique identifier
const byte ID = 1;
//begin message byte
byte BEGIN = 254; 

//Control byte
const byte SEND_MSG = 20;
const byte SEND_ERROR = 40;
const byte RECEIVE_MSG = 10;
const byte RECEIVE_MSG2 = 11;
//const byte RECEIVE_ERROR = 30;

//Flag Message byte
const byte ID_ID = 1;
const byte KA = 2;
const byte MODE = 10;
const byte MODE_CONF = 11;
const byte MEASINV = 20;
const byte GCS = 21;
const byte P = 30;
const byte Q = 40;
const byte P_CONF = 31;
const byte Q_CONF = 41;
const byte EXTRA = 100;
const byte BS = 110;
const byte BS_CONF = 111;
const byte MEAS_VDC = 120;

//Flag Error byte
const byte INV_ERROR = 10;

//TRUE / FALSE
const byte CONF_TRUE = 1;
const byte CONF_FALSE = 0;

//Timer const
const int COUNT_KA = 5;
const int COUNT_WM = 2;
const int COUNT_NM = 3;

//communication with raspberryPi
bool comm = false; 

//variables
byte input[64];
int first = 0;
int last = -1;

//timers
int count_ka = 0;
int count_wm = 0;
int count_nm = 0;

int ledPin = 13;
