//************************************************************************************************************************************************************************
// VARIABLES AND DEFINITIONS
//************************************************************************************************************************************************************************

//Includes libraries to perform specific tasks
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "g_cos_fix.h"
#include "h_pwm_due_lib.h"
// EnableInterrupt Simple example sketch. Demonstrates operation on a single pin of your choice.
// See https://github.com/GreyGnome/EnableInterrupt and the README.md for more information.
#include "i_enableInterrupt.h"
/*-------------------------------------------------------------------------------------------
  ARDUINO DUE PIN CONNECTIONS
  A0 - Grid Voltage
  A1 - Current Voltage
  A2 - DC Link Voltage

  D34 - PWM IGBT module 1 driver - TOP
  D35 - PWM IGBT module 1 driver - BOTTOM (complementary)
  D36 - PWM IGBT module 1 driver - TOP
  D37 - PWM IGBT module 1 driver - BOTTOM (complementary)

  D50 - Error IGBT module 1
  D51 - Error IGBT module 2
  D52 - Relay

  D12 - Led
*/
#define AMPS_INV A0  //the adc channel to read INVERTER amps
#define VOLTS_INV A1 //the adc channel to read INVERTER volts
#define VOLTS_DCLINK A2 //the adc channel to read DCLINK volts

#define Error_IGBT1 50
#define Error_IGBT2 51

#define INV_RELAY 53

#define LED_ 12

#define SYNC_PIN 71 //SDA1
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//DEFINITIONS
#define TURN_OFF_INV_RELAY digitalWrite(INV_RELAY, LOW);    // Turn off relay
#define TURN_OFF_LED digitalWrite(LED_, LOW);               // turn off led

#define TURN_ON_INV_RELAY digitalWrite(INV_RELAY, HIGH);    // turn on relay
#define TURN_ON_LED digitalWrite(LED_, HIGH);               // turn on led

//inverter mode
#define DCLINK_CTRL 1
#define ISLAND 2
#define BLACKSTART 3

static const int v_dc_ref = 220;
static const int v_ac_ref = 210; //Vrms

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//for measurement system
const float INV_V_scale = 254.723;// 274.771;//gain factor defined by the grid voltage sensor circuitry
const float INV_I_scale = 16.568;//15.951;
const float DC_V_scale = 106; //77.758;//78.055;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//for RT approach
static const float SAMPLERATE = 0.000260; //5kHz
static const long interval = 260;       //200 microseconds = SAMPLERATE
static const long interval_2 = 2000000; //for comunication 5sec

long duration = 0;
long duration2 = 0;
long durationSOGI = 0;
unsigned long previousMicros = 0;       // will store last time INVERTER
unsigned long previousMicros2 = 0;      // will store last time COMUNICATION
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//grid filter
static const float Lfilter = 0.010; //henry
static const float Rfilter = 0.1; //ohm 0.085 @25ºC

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//DECLARING GLOBAL VARIABLES
static const float _2PI = 6.283;
static constexpr float SOGI_W0 = 314.159;

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//for measurement system
float Vgrid = 0.0;
float Vgrid_max = 0.0;
float Igrid = 0.0;
float Igrid2 = 0.0;

float Vdclink = 0.0;

long GRID_CURRENT = 0;
long GRID_VOLTAGE = 0;
long BATTERY_CURRENT = 0;
long BATTERY_VOLTAGE = 0;
long DCLINK_VOLTAGE = 0;

long SHIFT_VOLTAGE = 2048;

//Filtered DClink
const float k1_fff = 0.005685; //0.000785;
const float k2_fff = -0.988629;; //-0.9984;
float Vdc_aux = 0.0;
float Vdc_aux_old = 0.0;
float Vdc_f = 0.0;

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//current control variables and vdclink control - DCLINK MODE
float id_ref = 0.0;
float iq_ref = 0.0;
//static const int v_dc_ref = 110;

float SOGI_angle = 0.0;
float SOGI_angle_IM = 0.0;
float theta = 0.0;

static const float K_INT_DC = 0.5; //0.8 //0.1
static const float K_PROP_DC = 0.05; //0.1 //0.01
static const int TERM_INT_MAX =20; 
static const int TERM_INT_MIN = -20; 
static const int id_ref_MAX = 10; 
static const int id_ref_MIN = -10;
float term_int = 0.0;

float v_dc_error = 0.0;

//grid voltage control variables for BLACKSTART MODE
//static const int v_ac_ref = 90; //sqrt(2)*80;

static const float K_INT_AC = 0.5;//0.2; //25;
static const float K_PROP_AC = 0.05;//0.008; //1;
static const int TERM_INT_MAX_2 =100; 
static const int TERM_INT_MIN_2 = -100;
static const int id_ref_MAX_2 = 10; 
static const int id_ref_MIN_2 = -10;
float term_int_ac = 0.0;
float term_prop_ac = 0.0;

float v_ac_error = 0.0;

float t_execution_ac = 0.0;

// variables for ISLAND MODe
static const int v_ac_ref_max = sqrt(2) * v_ac_ref; //Vpeak


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//ramp mode
static const int m_ramp = 20; // 20V/sec
float t_execution = 0.0;
bool ramp_mode = false; //for DCLINK control
bool ramp_mode3 = false; //for ISLAND mode

int v_ref_ctrl = 0;
int v_dc_initial = 0;

static const int m_ramp2 = 1; // for Q injection
float t_execution2 = 0.0;
bool ramp_mode2 = false;


int Iq_initial = 0;
int Iq_given = 0;

//time mode
static const int time_limit = 10; // 
bool time_mode = false;
float t_execution3 = 0.0;

//time_mode_DCLINK
static const float time_limit_DCLINK = 2;
bool time_mode_DCLINK = false;
float t_execution_DCLINK = 0.0;

//external sync
int sync_pulses_number = 0;
long sync_old_time = 0;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//PWM limit
int PWM_MAX = 0;
int PWM_SUP = 0;
int PWM_INF = 0;

float DC_max = 0.95;
float DC_min = 0.05;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//FLAGS
//limits
//Vint = (Vfloat/scale)*(4095/3.3)
static const int Vinv_max = (v_ac_ref * 1.40 * sqrt(2) /INV_V_scale)*(4095/3.3);//276
static const int Vinv_min = (v_ac_ref * 0.40 * sqrt(2) /INV_V_scale)*(4095/3.3);//184
static const int Iinv_max = (30*sqrt(2) /INV_I_scale)*(4095/3.3);               //16Arms is the maximum, limit could be defiend as sqrt(2)*16 = 22.6A

static const int Vdclink_max = v_dc_ref * 1.14; //200;//(264 /DC_V_scale)*(4095/3.3);
static const int Vdclink_min = v_dc_ref * 0.50; //80;//(176 /DC_V_scale)*(4095/3.3);
static const int Vdclink_max2 = v_dc_ref * 1.53; //200;//(264 /DC_V_scale)*(4095/3.3);

static const int Vd_max = v_ac_ref * 1.40 *sqrt(2);
static const int Vd_min = v_ac_ref * 0.40 *sqrt(2);
static const int W_max = 330;
static const int W_min = 298;

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//control variables
bool INV_connected = false; //if the battery is connected to the system - relay on
bool INV_connecting = false;
bool INV_disconnecting = false;
bool COM_established = false;

bool led_blinking = false;
int led_state = 0;

bool block_all =  false;
bool inv_ctrl = false;

//others variables
unsigned long INV_reset = 0;

//inverter mode : check definitions
int INVERTER_mode = 1;

bool island_mode_achived =  false;
bool grid_avaliable = false;

//grid return detection
int peak_detection = 0;
float id_ref_old = 0.0;

int n_errors = 0;
int n_errors2 = 0;

int n_limits_error = 0; //in order do not stop the inverter if a peak was detected
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//POWERs calculations
static int array_size = 19; //5ms / samplarate (90º)
float I_ARRAY[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float V_ARRAY[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//Vrms calculation
long root_mean_square_voltage = 0;
int rms_rate = 0;
static const int array_size2 = 20;
float V_ARRAY_RMS[20];

//Filter
const float k1_f = 0.005685;
const float k2_f = -0.988629;

float Pmeas = 0.0;
float Pmeas_aux = 0.0;
float Pmeas_aux_old = 0.0;
float Pmeas_f = 0.0;

float Qmeas = 0.0;
float Qmeas_aux = 0.0;
float Qmeas_aux_old = 0.0;
float Qmeas_f = 0.0;
