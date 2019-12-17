/* --------------------------------------------------------------------------------------
 * Functions
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
 * Description: This file includes libraries to perform specific tasks
  * 
-----------------------------------------------------------------------------------------**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "g_cos_fix.h"
#include "h_pwm_due_lib.h"
/*-------------------------------------------------------------------------------------------
  ARDUINO DUE PIN CONNECTIONS
  A0 - Battery Current
  A1 - Battery Voltage
  A2 - PV Current
  A3 - PV Voltage
  A4 - DC Link Voltage

  D34 - PWM IGBT module 1 (PV) driver - TOP
  D35 - PWM IGBT module 1 
  
  D36 - PWM IGBT module 2 (BAT) driver - BOTTOM
  D37 - PWM IGBT module 2 (BAT) driver - TOP (complementary)

  
  D49 - Error INPUT from IGBT module 2 (ESS)
  D50 - Error INPUT form IGBT module 1 (PV)

  D51 - IGBT LOAD reset signal
  D52 - Switch LOAD
  D53 - IGBT LOAD error signal

  D12 - Notification LED

*/
#define AMPS_BAT A0  //the adc channel to read INVERTER amps
#define VOLTS_BAT A1 //the adc channel to read INVERTER volts
#define AMPS_PV A2  //the adc channel to read INVERTER amps
#define VOLTS_PV A3 //the adc channel to read INVERTER volts
#define VOLTS_DCLINK A4 //the adc channel to read DCLINK volts

#define Error_IGBT1 50 //for PV
#define Error_IGBT2 49 //for BAT

#define Error_IGBT3 53 // DRIVER of LOAD IGBT
#define Reset_IGBT3 51 // DRIVER of LOAD IGBT
#define DC_LOAD_S 52

#define LED_ 12

//convertor number
#define C_BAT 1
#define C_PV 2
#define C_DCLOAD 3


//*******************************************************************************************
//DEFINITIONS
//*******************************************************************************************
#define TURN_OFF_LED digitalWrite(LED_, LOW);               // turn off led
#define TURN_ON_LED digitalWrite(LED_, HIGH);               // turn on led

#define TURN_ON_DC_SWITCH digitalWrite(DC_LOAD_S, HIGH);         // turn on
#define TURN_OFF_DC_SWITCH digitalWrite(DC_LOAD_S, LOW);         // turn on


int Vdclink_ref = 220;//battery voltage reference

const float baterry_capacity = 17;
int nbat = 8;

static const int P_PV_min = 30;
static const int V_PV_min = 50; //60% Voc

//CHANGE LIMITS BELOW

//*******************************************************************************************
//DEFINITIONS & DECLARING GLOBAL VARIABLES
//*******************************************************************************************
static const float _2PI = 6.283;

//-------------------------------------------------------------------------------------------
//for measurement system
const float BAT_V_scale = 47.510;//gain factor defined by the grid voltage sensor circuitry
const float BAT_I_scale = 16.344;
const float PV_V_scale = 46.976;
const float PV_I_scale = 7.857;
const float DC_V_scale = 106; //103;// 77.758; 82.5

float Vbat = 0.0;
float Ibat = 0.0;
float Vpv = 0.0;
float Ipv = 0.0;
float Vdclink = 0.0;

float Vbat_meas_max = 0.0;
float Vbat_meas_min = 0.0;
float Ibat_meas_max = 0.0;

long BATTERY_CURRENT = 0;
long BATTERY_VOLTAGE = 0;
long PV_CURRENT = 0;
long PV_VOLTAGE = 0;
long DCLINK_VOLTAGE = 0;

long SHIFT_VOLTAGE = 2048;

//-------------------------------------------------------------------------------------------
//Definitions for RT approach
static const float SAMPLERATE = 0.000300; //5kHz
static const long interval = 300;       //for battery task
static const long interval_2 = 2000000;     //for PV task 3sec
static const long interval_3 = 10000000; //for SOC 10sec
static const long interval_4 = 2000000; //for comunication task 2sec
long duration = 0;
long duration2 = 0;
long duration3 = 0;
long duration4 = 0;
unsigned long previousMicros = 0;       // will store last time INVERTER
unsigned long previousMicros2 = 0;      // will store last time BATTERY
unsigned long previousMicros3 = 0;      // will store last time SOC
unsigned long previousMicros4 = 0;      // will store last time COMUNICATION
  
//-------------------------------------------------------------------------------------------
//Battery Converter

int BAT_mode = 1; //1: Controlling DC link / 2: controlling power injjection

//definitions for battery PI voltage control
static const float BAT_Kp = 0.05;
//static const float BAT_Ti = 0.01;
static const float BAT_Ki = 1.0; //Kp/Ti;
//definitions for battery PI current control
static const float BAT_Kp2 = 0.08;
//static const float BAT_Ti = 0.01;
static const float BAT_Ki2 = 2.0; //SOGI_Kp/SOGI_Ti;

//definitions for battery PI current control (power injection mode 2)
static const float BAT_Kp3 = 0.06;
//static const float BAT_Ti = 0.01;
static const float BAT_Ki3 = 3.0; //SOGI_Kp/SOGI_Ti;

//variables for battery PI current control
float BAT_I_EI = 0.0;
float BAT_I = 0.0;
float V_error = 0.0;
float V_error_0 = 0.0;
//variablesfor battery PI voltage control
float BAT_I_EI2 = 0.0;

//others variables
float BAT_DC = 0.0;
int DC_to_PWM_BAT = 0;
float I_error = 0.0;

//for ramp mode
static const int m_ramp = 30; //10V/sec for  DClink ramp
static const int m_ramp2 = 1; //1A/sec for  current control ramp
int m_ramp2_aux = 1;
int t_ramp2 = 0;

float t_execution = 0.0;
int Vbat_initial = 0;
float Ibat_initial = 0.0;
bool ramp_mode = false;
bool ramp_conf =  false; //for battery current control

int Pbat_ref = 0;

int Vdclink_ref_ctrl = 0;
//int Vdclink_ref = 110;//battery voltage reference

float Ibat_ref = 0.0;
float Ibat_ref_ctrl = 0;

static const float Lfilter_bat = 0.005; //henry
static const float Rfilter_bat = 0.1; //ohm 0.085 @25ºC
//-------------------------------------------------------------------------------------------
//PV Converter
int DC_PV = 0; //testar convertidor PV
int P_PV_ref = 0;
int PV_mode = 0; //1: DC / 2: RPPT / 3: MPPT

int PV_PWM_MAX = 0;                  // the value for pwm duty cyle 0-100%
int PV_PWM_MIN = 0;                  // the value for pwm duty cyle 0-100%
int PV_PWM_START = 0;                // the value for pwm duty cyle 0-100%

float old_Vpv = 0; //valor real do transdutor
float old_Ipv = 0;
float old_Ppv = 0;
int old_PV_PWM_DC = 0;

int PV_PWM_INC_ARRAY[] = {0,0,0,0,0,0};

//for PV MPPT control
int PV_PWM_INC_MPPT_INITIAL = 1;
int PV_PWM_INC_MPPT = 1;
int PV_PWM_INC_MPPT_MIN = 1;

//for RPPT
int PV_PWM_INC_RPPT_INITIAL = 1;
int PV_PWM_INC_RPPT_MIN = 1;
int PV_PWM_INC_RPPT = 1;
int PV_P_REF_PASS = 0;
int PV_PWM_INC_POSITIVE_DIRECTION = 0;
int PV_PWM_INC_NEGATIVE_DIRECTION = 0;
static const int V_mpp_t = 55;

//for the SOC estimation***********************************************************************
const float Cmax = baterry_capacity; //7.0; //Maximum capacity of the battery in Ah. This value will depend on the selected battery
const float SOC_P_min = 20.0; //Minimum % of capacity for the battery. Safety margin. 
const float SOC_P_max = 80.0; //Maximum % of capacity for the battery. Safety margin.

float SOC_0 = 0*Cmax/100; //3.5; //Initial state of charge in Ah. This value will depend on the battery capacity and will be updated in each cycle
float SOC = 0.0; //Current state of charge in Ah
float SOC_P = 0.0; //Current state of charge in %

bool SOC_0_not_initialized = true;

int ESS_resting_time = 180;//intervalo de 10 segundos, bateria descansar 30 min
int bat_resting = ESS_resting_time;

float Ibat_standbye = 0.5;


//definitions for PV PI current control
static const float PV_Kp = 0.06;
//static const float PV_Ti = 0.01;
static const float PV_Ki = 3.0; //SOGI_Kp/SOGI_Ti;

//variables for battery PI current control
float PV_I_EI = 0.0;
float PV_I = 0.0;

float t_execution_pv = 0.0;
float Ipv_initial = 0.0;
bool ramp_mode_pv = true;
int Ipv_ref_new = 0; //current
float Ipv_ref_ctrl = 0.0;

//-------------------------------------------------------------------------------------------
//others variables
int PWM_MAX = 0; //PWM resolution
int PWM_LIM_SUP = 0;
int PWM_LIM_INF = 0;

//Filtered powers
// for BAT CURRENT
const float k1_f = 0.006555;
const float k2_f = -0.98689;
float Ibat_aux = 0.0;
float Ibat_aux_old = 0.0;
float Ibat_f = 0.0;

//FOR BAT VOLTAGE
const float k1_bat_v = 0.006555;
const float k2_bat_v = -0.98689;
float Vbat_aux = 0.0;
float Vbat_aux_old = 0.0;
float Vbat_f = 0.0;

//FOR PV VOLTAGE
const float k1_pv = 0.75476;
const float k2_pv = 0.50953;
float Vpv_aux = 0.0;
float Vpv_aux_old = 0.0;
float Vpv_f = 0.0;
//-------------------------------------------------------------------------------------------
//FLAGS
//Vint = (Vfloat/scale)*(4095/3.3)
static const int Vbat_max = nbat*12*1.25; //(120 /BAT_V_scale)*(4095/3.3);
static const int Vbat_min = nbat*12*0.75; //(70 /BAT_V_scale)*(4095/3.3);

static const int Ibat_max = 20; //(25 /BAT_I_scale)*(4095/3.3);
static const int Ibat_min = -20; //(-25 /BAT_I_scale)*(4095/3.3);
static const int Vpv_max = (150 /PV_V_scale)*(4095/3.3);
static const int Vpv_min = (20 /PV_V_scale)*(4095/3.3);
static const int Ipv_max = (20 /PV_I_scale)*(4095/3.3);
static const int Ipv_min = (0 /PV_I_scale)*(4095/3.3);

static const int Vdclink_max = ( (Vdclink_ref * 1.12) /DC_V_scale)*(4095/3.3);//320
static const int Vdclink_min = ( (Vdclink_ref * 0.50) /DC_V_scale)*(4095/3.3);//120
static const int Vdclink_max2 = ( (Vdclink_ref * 1.53) /DC_V_scale)*(4095/3.3);//320


static const int Ibat_max_PI = 18;
static const int Ibat_min_PI = -6;

long n_limits_error = 0;

//LIMITS for set functions
int I_bat_ref_max = 18;
int I_bat_ref_min = -18;

int P_pv_ref_max = 1000;
int P_pv_ref_min = 50;

//-------------------------------------------------------------------------------------------
//control variables
bool BAT_connected = false; //if the battery is connected to the system - relay on
bool BAT_connecting = false;
bool BAT_disconnecting = false;
bool PV_connected = false; //if the pv is connected to the system - relay on
bool PV_connecting = false;
bool PV_disconnecting = false;
bool DC_connected = false; //if the LOADs is connected to the system - relay on
bool DC_connecting = false;
bool DC_disconnecting = false;

bool led_blinking = false;
int led_state = 0;

bool block_BAT =  false;
bool block_DCLOAD = false;
bool block_PV = false;

long Vbat_mean_a[] = {0,0,0,0,0,0,0,0,0,0};
long Vbat_mean = 0;

bool flag_delay = false; //this variable will be used to add a delay for the flag limits checking
long flag_counting = 0;
