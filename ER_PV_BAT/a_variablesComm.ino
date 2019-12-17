/* --------------------------------------------------------------------------------------
 * Communication variables
 * --------------------------------------------------------------------------------------
 * Copyright (c) 2019 Nuno Vilhena <nuv@uninova.pt>
 *
 * This file is part of ER PV BAT.
 *
 * ER PV BAT is free software; you can redistribute it and/or
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
 * License along with ER PV BAT. If not, see: <http://www.gnu.org/licenses/>.
 * --------------------------------------------------------------------------------------
 *
 * Description: This file includes the communication variables
  * 
-----------------------------------------------------------------------------------------**/


//debug
//#define DEBUG

//unique identifier
const byte ID = 2;
//begin message byte
byte BEGIN = 254; 

//Control byte
const byte SEND_MSG = 21;
const byte SEND_ERROR = 41;
const byte RECEIVE_MSG = 11;
const byte RECEIVE_MSG2 = 10;
//const byte RECEIVE_ERROR = 31;

//Flag Message byte
const byte ID_ID = 1;
const byte KA = 2;
const byte MODEBAT = 50;
const byte MODEBAT_CONF = 51;
const byte MODEPV = 70;
const byte MODEPV_CONF = 71;
const byte BAT = 60;
const byte PV = 80;
const byte BAT_CONF = 61;
const byte PV_CONF = 81;
const byte LOADS = 90;
const byte LOADS_CONF = 91;
const byte EXTRA = 100;
const byte BS = 110;
const byte BS_CONF = 111;

//Flag Error byte
const byte BAT_ERROR = 20;
const byte PV_ERROR = 30;

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
