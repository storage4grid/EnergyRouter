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
 * Description: This file includes the routine to manage IGBT errors (received from ERROR_OUT of IGBTs
  * 
-----------------------------------------------------------------------------------------**/
//-------------------------------------------------------------------------------------------
  void Error_Treatment_BAT(){
//    Serial.println("ERROR in BAT driver");
    delayMicroseconds(10);
    if(digitalRead(Error_IGBT2)){  
      shutdown_();
      led_blinking = true;
      //block arduino until reset
      block_BAT =  true;
      serialSendBatError(3);
    }
    //send a error msg to master in order to block the drivers
  }

  void Error_Treatment_PV(){
//    Serial.println("ERROR in PV_DCLINK driver");
    delayMicroseconds(10);
    if(digitalRead(Error_IGBT1)){  
      PV_disconnect_();
      led_blinking = true;
      block_PV =  true;
      serialSendPvError(2);
    }
    //send a error msg to master in order to block the drivers
  }

    void Error_Treatment_DCLOAD(){
//    Serial.println("ERROR in PV_DCLINK driver");
    delayMicroseconds(10);
    if(!digitalRead(Error_IGBT3)){
      delayMicroseconds(10);
      if(!digitalRead(Error_IGBT3)){  
        TURN_OFF_DC_SWITCH
        led_blinking = true;
        block_DCLOAD =  true;
        serialSendBatError(4);
        //shutdown_();
      }
    }
  }

   void Error_Treatment2(int type){
    if(type == C_BAT){
//      Serial.println("Out of Ranges");
      BAT_disconnect_();
      serialSendBatError(1);
      serialSendExtra((float)SOC_P, Vbat_f, Ibat_f);
      
    }else if(type == C_PV){
//      Serial.println("Out of Ranges");
      PV_disconnect_();
      serialSendPvError(1);
      serialSendExtra(0, Vpv, Ipv);
      
    }else if(type == C_DCLOAD){
//      Serial.println("Out of Ranges");
      DC_disconnect_();
      //serialSendExtra(float value1, float value2, float value3)
    }
    led_blinking = true;
  }
  
//-------------------------------------------------------------------------------------------
// This routine shutdown the system
//-------------------------------------------------------------------------------------------
  void shutdown_(void){
   
    TURN_OFF_DC_SWITCH;
    stop_pwm();

    //TURN_ON_LED;
    led_blinking = true;

    BAT_connected = false;
    BAT_connecting = false;
    PV_connected = false;
    PV_connecting = false;
    DC_connected = false;
    DC_connecting = false;

    //sending error code to RBpi
  }

//-------------------------------------------------------------------------------------------
// This routine disconnect the battery
//-------------------------------------------------------------------------------------------
  void BAT_disconnect_(void){   
    stop_pwm(2);
    led_blinking = false;
    TURN_ON_LED;

    BAT_connected = false;
    BAT_disconnecting = false;
  }
//-------------------------------------------------------------------------------------------
// This routine disconnect the PV
//-------------------------------------------------------------------------------------------
  void PV_disconnect_(void){  

    stop_pwm(1);
    led_blinking = false;

    PV_PWM_INC_MPPT = PV_PWM_INC_MPPT_INITIAL;
    PV_PWM_INC_RPPT = PV_PWM_INC_RPPT_INITIAL;
       
    PV_connected = false;
    PV_disconnecting = false;
    

  }
//-------------------------------------------------------------------------------------------
// This routine disconnect the DC link
//-------------------------------------------------------------------------------------------
  void DC_disconnect_(void){    
    TURN_OFF_DC_SWITCH;

    DC_connected = false;
    DC_disconnecting = false;
  }
  
//-------------------------------------------------------------------------------------------
// This routine connect the battery to the system
//-------------------------------------------------------------------------------------------
  void BAT_connect_(void){ 
    //RESET VARIABLES
    BAT_I_EI = 0;
    BAT_I_EI2 = 0;
   
    //delay(1); //delay of 1msec just to stabilize
    led_blinking = false;
    TURN_OFF_LED;
    
    BAT_connected = true;
    BAT_connecting = false;

    //add a delay before check the flags limits
    flag_delay = true;
    flag_counting = 100000 / interval; //delay of 100 ms = 100000 usec

    //TURN_ON_BAT
    start_pwm(2);

  }
  
//-------------------------------------------------------------------------------------------
// This routine connect the PV to the system
//-------------------------------------------------------------------------------------------
  void PV_connect_(void){
    //reset PV variables
    DC_PV = 0;
    old_PV_PWM_DC = PV_PWM_START;
    PV_PWM_INC_MPPT = PV_PWM_INC_MPPT_INITIAL;

    old_Ipv=0;
    old_Vpv=0;
    old_Ppv = 0;

    //reset variables current control*************************
    PV_I_EI = 0;
    
    //delay(1); //delay of 1msec just to stabilize  

    led_blinking = false;
    TURN_OFF_LED;
    PV_connected = true;
    PV_connecting = false;
    start_pwm(1);
  }
//-------------------------------------------------------------------------------------------
// This routine connect the DCLINK to the system
//-------------------------------------------------------------------------------------------
  void DC_connect_(void){

    if(!block_DCLOAD){
    TURN_ON_DC_SWITCH;

    DC_connected = true;
    DC_connecting = false;
    }
    
  }
  
//-------------------------------------------------------------------------------------------
// Those routines calculate cos and sin
//-------------------------------------------------------------------------------------------
  float cos_fix_rad(float x)
  {
    static const int max_in = (1UL << 16);
    static const int max_out = (1UL << 14);
    static const float _2PI = 6.283;

    int aux = cos_fix(x * max_in / _2PI);

    return (float)aux / max_out;
  }

  float sin_fix_rad(float x)
  {
    static const int max_in = (1UL << 16);
    static const int max_out = (1UL << 14);
    static const float _2PI = 6.283;

    int aux = sin_fix(x * max_in / _2PI);

    return (float)aux / max_out;
  }
//-------------------------------------------------------------------------------------------
// This routine compare two arrays
//-------------------------------------------------------------------------------------------
  boolean array_cmp(int *a, int *b, int len_a, int len_b){
     int n;

     // if their lengths are different, return false
     if (len_a != len_b) return false;

     // test each element to be the same. if not, return false
     for (n=0;n<len_a;n++) if (a[n]!=b[n]) return false;

     //ok, if we have not returned yet, they are equal :)
     return true;
}

//*******************************************************************************************
// COMMUNICATION FUNCTIONS
//*******************************************************************************************
void receiveSoCfile(float insoc){
    SOC_0=insoc * Cmax/100;
    SOC_P = insoc;
     //SOC_0=60 * Cmax/100;
}

void receivePv_OFF(){
  
  if(PV_connected == true)
    PV_disconnecting = true;
}

void receivePv_MPPT(){
 
  PV_mode = 1;
  PV_connecting = true;

}

void receivePv_RPPT(float ppv){

//  if(PV_current_control){
//  //for current control 
//    ramp_mode_pv = true;
//    Ipv_initial = Ipv;
//    
//    
//    Ipv_ref_new = ppv / Vpv;
//    t_execution_pv = 0;
//    PV_mode = 3;
//    PV_connecting = true;
//  }
//  else{ 
//    
//  
  PV_mode = 2;
  PV_connecting = true;

  P_PV_ref = (int)ppv;

  if(P_PV_ref > P_pv_ref_max)
    P_PV_ref = P_pv_ref_max;
  if(P_PV_ref < P_pv_ref_min)
    P_PV_ref = P_pv_ref_min;

  //PV_PWM_INC_RPPT = PV_PWM_INC_RPPT_INITIAL;
}

void receiveBat_OFF(){
  BAT_disconnecting = true;
  
  //set ramp mode if bat is in current control mode
  if(BAT_mode == 2){
    ramp_mode = true;
    ramp_conf = true;
    Ibat_initial = Ibat_ref;
    Ibat_ref = 0;
    t_execution = 0;
  }
}

void receiveBat_SBY(){
  if(BAT_connected == false){
    BAT_mode = 1;
    BAT_connecting = true;
             
    //set ramp mode
    ramp_mode = true;
    Vbat_initial = (int)Vdclink;
    t_execution = 0;
  }
}

void receiveBat_P(float pbat){
    Pbat_ref =  pbat;
    if(BAT_connected == false){
      Ibat_initial = 0;
      Ibat_ref = pbat/Vbat_f;
      if(Ibat_ref > I_bat_ref_max)
        Ibat_ref = I_bat_ref_max;
      if(Ibat_ref < I_bat_ref_min)
        Ibat_ref = I_bat_ref_min;
        
      //check SOC  
      if( (round(SOC_P) < SOC_P_max && Ibat_ref < 0) || (round(SOC_P) > SOC_P_min && Ibat_ref > 0) ){   
        ramp_mode = true;
        ramp_conf = true;
        t_execution = 0;

        BAT_mode = 2;
        BAT_connecting = true;
      }
    }
    else
    if(BAT_connected == true && BAT_mode == 2){
      
      Ibat_initial = Ibat_ref;
      Ibat_ref = pbat/Vbat_f;
      if(Ibat_ref > I_bat_ref_max)
        Ibat_ref = I_bat_ref_max;
      if(Ibat_ref < I_bat_ref_min)
        Ibat_ref = I_bat_ref_min;

      ramp_mode = true;
      ramp_conf = true;
      t_execution = 0;
    }
}

void receiveLoad(boolean value){
  if(value == true)
    DC_connect_();

  else
    DC_disconnect_();
}

void receiveBS(){
}

void commLost(){
  PV_disconnecting = true;
  DC_disconnect_();
  BAT_disconnecting = true;
  led_blinking = true;
}
