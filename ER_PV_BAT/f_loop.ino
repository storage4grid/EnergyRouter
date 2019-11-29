//*******************************************************************************************
// Main loop.
//*******************************************************************************************
void loop() {
  unsigned long currentMicros = micros(); //initial time;

//-------------------------------------------------------------------------------------------
//TASK for BATTERY
//-------------------------------------------------------------------------------------------
  if (currentMicros - previousMicros >= interval) {

/////////////////////////////////////////////////////////////////////////////////////////////
//MEASUREMENTS
    //read battery current from analogue port*******************************************
    BATTERY_CURRENT = analogRead(AMPS_BAT);
    BATTERY_CURRENT = BATTERY_CURRENT - SHIFT_VOLTAGE; //retira o offset e adiciona correccao por causa do filtro adicionado
    //Ibat = (float)BATTERY_CURRENT * 3.3 / 4095.0;  //check if this calue is the right one
    //Ibat = Ibat * BAT_I_scale;
    Ibat = BATTERY_CURRENT * 0.013171;

    Ibat_aux = Ibat * k1_f - Ibat_aux_old * k2_f;
    Ibat_f = Ibat_aux + Ibat_aux_old;
    Ibat_aux_old = Ibat_aux;

    //read battery voltage from analogue ports*******************************************
    BATTERY_VOLTAGE = analogRead(VOLTS_BAT);

    Vbat = BATTERY_VOLTAGE * 0.038286;

    Vbat_aux = Vbat * k1_bat_v - Vbat_aux_old * k2_bat_v;
    Vbat_f = Vbat_aux + Vbat_aux_old;
    Vbat_aux_old = Vbat_aux;

    //read DCLINK voltage from analogue ports*******************************************
    DCLINK_VOLTAGE = analogRead(VOLTS_DCLINK);
    //Vdclink = (float)DCLINK_VOLTAGE * 3.3 / 4095.0; //conversao ppois está a ser utilizado 12 bits que da um maximo de 4096 para 3.3V
    //Vdclink = Vdclink * DC_V_scale;
    Vdclink = DCLINK_VOLTAGE * 0.085421;

    //connect the battery converter
    if (BAT_connecting && block_BAT == false)
      BAT_connect_();
    if (BAT_disconnecting && BAT_mode == 1) //if bat is controlling DClink, just disconnect
      BAT_disconnect_();

    if (flag_delay) {
      flag_counting--;
      if (flag_counting <= 0)
        flag_delay = false;
    }

/////////////////////////////////////////////////////////////////////////////////////////////
//CHECK FLAGS DC link

    if (BAT_connected || PV_connected)
      if (DCLINK_VOLTAGE >= Vdclink_max || (DCLINK_VOLTAGE < Vdclink_min && ramp_mode == false))
      {
        n_limits_error++;
        if (n_limits_error > 5000 || DCLINK_VOLTAGE >= Vdclink_max2) { //2000 = 0.5sec
          Error_Treatment2(C_DCLOAD);
          Error_Treatment2(C_PV);
          Error_Treatment2(C_BAT);
          serialSendExtra(Vdclink, 0, 0);
          n_limits_error = 0;
        }
      }
      else
        n_limits_error = 0;

/////////////////////////////////////////////////////////////////////////////////////////////
//BATTERY CONNECTION AND CONTROL ALGORITHM
    if (BAT_connected) {

/////////////////////////////////////////////////////////////////////////////////////////////
//CHECK FLAGS FOR BATTERY

      if ( ( (Vbat_f >= Vbat_max || Vbat_f <= Vbat_min) && flag_delay == false) || Ibat_f >= Ibat_max || Ibat_f <= Ibat_min)
      {
        //wait 100 ms until turn of the converter, in order to avoid peaks due the turn on the converter
        if (BAT_mode == 1) { //controling DCLINK
          Error_Treatment2(C_DCLOAD);
          Error_Treatment2(C_PV);
          Error_Treatment2(C_BAT);
        }
        else { //controling POWER
          //Error_Treatment2(3);
          //Error_Treatment2(2);
          Error_Treatment2(C_BAT);
        }

        //serialSendExtra(Ibat, Vbat_f, Vbat);
      }

/////////////////////////////////////////////////////////////////////////////////////////////
//CHECK SOC VALUE

      //this is when ER is working in GRID OFF.
      if (SOC_P > SOC_P_max && Ibat_f < -Ibat_standbye && BAT_mode == 1) { //controling DC LINK
        receivePv_OFF();
        TURN_OFF_DC_SWITCH;
        //receiveBat_OFF();
        serialSendBatError(2);
      }
      else if (SOC_P < SOC_P_min && Ibat_ref > Ibat_standbye && BAT_mode == 1) { //controling DC LINK
        receivePv_OFF();
        TURN_OFF_DC_SWITCH;
        serialSendBatError(2);
      }
      //this is when ER is working in GRID ON.
      else if (SOC_P > SOC_P_max && Ibat_ref < 0 && BAT_mode == 2) { //RECEIVING POWER
        receiveBat_OFF();
        serialSendBatError(2);
      }
      else if (SOC_P < SOC_P_min && Ibat_ref > 0 && BAT_mode == 2) { //INJECTING POWER
        receiveBat_OFF();
        serialSendBatError(2);
      }
/////////////////////////////////////////////////////////////////////////////////////////////
//CONTROL ALGORITHMS

      //CONTROLS THE DCLINK VOLTAGE
      if (BAT_mode == 1) {

        //CONTROLO PARA BUCK BOOST BIDIRECCIONAL*****************************************************
        //apply a ramp function to Vdclink_ref
        if (ramp_mode) {
          Vdclink_ref_ctrl = m_ramp * t_execution + Vbat_initial;
          //increment
          if (Vdclink_ref_ctrl < Vdclink_ref)
            t_execution = t_execution + SAMPLERATE;
          else
          {
            ramp_mode = false;
            Vdclink_ref_ctrl = Vdclink_ref;
          }
        }

        V_error = Vdclink_ref_ctrl - Vdclink;

        //---------- VOLTAGE LOOP PI WITH BACKWARD EULER INTEGRATOR----------------------------------
        BAT_I_EI = BAT_I_EI + SAMPLERATE * BAT_Ki * V_error;
        BAT_I = BAT_I_EI + BAT_Kp * V_error;

        if (BAT_I < Ibat_min_PI)
          BAT_I = Ibat_min_PI;
        else if (BAT_I > Ibat_max_PI)
          BAT_I = Ibat_max_PI;

        I_error =  BAT_I - Ibat;

        //---------- CURRENT LOOP PI WITH BACKWARD EULER INTEGRATOR----------------------------------
        BAT_I_EI2 = BAT_I_EI2 + SAMPLERATE * BAT_Ki2 * I_error;
        BAT_DC = BAT_I_EI2 + BAT_Kp2 * I_error;
        BAT_DC = BAT_DC * PWM_MAX;
        DC_to_PWM_BAT = (int)BAT_DC;

        if (DC_to_PWM_BAT < PWM_LIM_INF)
          DC_to_PWM_BAT = PWM_LIM_INF;
        else if (DC_to_PWM_BAT > PWM_LIM_SUP)
          DC_to_PWM_BAT = PWM_LIM_SUP;

      }// end  CONTROLS THE DCLINK VOLTAGE
      else

/////////////////////////////////////////////////////////////////////////////////////////////
//CONTROLS POWER INJECTION
        if (BAT_mode == 2) {

          //apply a ramp function to Vdclink_ref
          if (ramp_mode) {
            if (ramp_conf)
            {
              if ((Ibat_ref < 0 && Ibat_initial >= 0) || (Ibat_ref == 0 && Ibat_initial >= 0))
                m_ramp2_aux = - m_ramp2;
              else
                m_ramp2_aux = m_ramp2;

              t_ramp2 = (Ibat_ref - Ibat_initial) / m_ramp2_aux;
              ramp_conf =  false;
            }

            Ibat_ref_ctrl = m_ramp2_aux * t_execution + Ibat_initial;

            if (Ibat_ref_ctrl < Ibat_min_PI) {
              Ibat_ref_ctrl = Ibat_min_PI;
              ramp_mode =  false;
            }
            else if (Ibat_ref_ctrl > Ibat_max_PI) {
              Ibat_ref_ctrl = Ibat_max_PI;
              ramp_mode =  false;
            }
            //increment

            if (t_execution < t_ramp2)
              t_execution = t_execution + SAMPLERATE;
            else
            {
              ramp_mode = false;
              Ibat_ref_ctrl = Ibat_ref;

              if ((Ibat_ref_ctrl == 0 && BAT_disconnecting == true) || (Ibat_ref_ctrl == 0 && Ibat_ref == 0)) {
                BAT_disconnect_();
              }
            }
          }


          I_error =  Ibat_ref_ctrl - Ibat;

//---------- CURRENT LOOP PI WITH BACKWARD EULER INTEGRATOR------------------------------
          BAT_I_EI2 = BAT_I_EI2 + SAMPLERATE * BAT_Ki3 * I_error;
          BAT_DC = BAT_I_EI2 + BAT_Kp3 * I_error;
          BAT_DC = BAT_DC * PWM_MAX;
          DC_to_PWM_BAT = (int)BAT_DC;

          if (DC_to_PWM_BAT < PWM_LIM_INF)
            DC_to_PWM_BAT = PWM_LIM_INF;
          else if (DC_to_PWM_BAT > PWM_LIM_SUP)
            DC_to_PWM_BAT = PWM_LIM_SUP;
        }

      updatePWM( 2, DC_to_PWM_BAT);
    }

//testing task duration
    duration = micros() - currentMicros;
    previousMicros = currentMicros;
  }


//-------------------------------------------------------------------------------------------
//TASK for PV
//-------------------------------------------------------------------------------------------
  if (currentMicros - previousMicros2 > interval_2) {
/////////////////////////////////////////////////////////////////////////////////////////////
//MEASUREMENTS

//read PV current from analogue port*********************************************************
    PV_CURRENT = analogRead(AMPS_PV);
    //Ipv = (float)PV_CURRENT * 3.3 / 4095.0;  //check if this calue is the right one
    //Ipv = Ipv * PV_I_scale;
    Ipv = PV_CURRENT * 0.0063316;

//read PV voltage from analogue ports********************************************************
    PV_VOLTAGE = analogRead(VOLTS_PV);
    //Vpv = (float)PV_VOLTAGE * 3.3 / 4095.0; //conversao ppois está a ser utilizado 12 bits que da um maximo de 4096 para 3.3V
    //Vpv = Vpv * PV_V_scale;

    Vpv = (float)PV_VOLTAGE * 0.037856;

    Vpv_aux = Vpv * k1_pv - Vpv_aux_old * k2_pv;
    Vpv_f = Vpv_aux + Vpv_aux_old;
    Vpv_aux_old = Vpv_aux;

/////////////////////////////////////////////////////////////////////////////////////////////
//connect the PV converter
    if (PV_connecting && block_PV == false) //if the inverter is sync with grid and the flag inverter_connecting is true
      PV_connect_();

    if (PV_disconnecting) { //if the inverter is sync with grid and the flag inverter_connecting is true
      //reduce the dutycycle until zero before turn off the converter
      PV_connected = false;
      float P_pv = Ipv * Vpv_f;
      int PV_PWM_DC = old_PV_PWM_DC - PV_PWM_INC_MPPT_INITIAL;
      if (PV_PWM_DC <= 0 || P_pv < 200) {
        PV_disconnect_();
      }
      else
      {
        updatePWM( 1, PV_PWM_DC);
        //save old value
        old_PV_PWM_DC = PV_PWM_DC;
      }
    }

/////////////////////////////////////////////////////////////////////////////////////////////
//CONNECT PV
    if (PV_connected) {

/////////////////////////////////////////////////////////////////////////////////////////////
//CHECK FLAGS
      //  if(PV_VOLTAGE >= Vpv_max || PV_VOLTAGE <= Vpv_min || PV_CURRENT >= Ipv_max )
      //    Error_Treatment2(C_PV);

/////////////////////////////////////////////////////////////////////////////////////////////
//CONTROL ALGORITHMS
//CONTROLO PARA BOOST CONVERTER MPPT

//implement P&O method ----------------------------------------------------------------------
      if (PV_mode == 1) {

        int PV_PWM_DC;

        float Ppv = Ipv * Vpv;
        old_Ppv = old_Ipv * old_Vpv;


        if (Ppv < P_PV_min && Vpv > V_PV_min)
          PV_PWM_DC = old_PV_PWM_DC + PV_PWM_INC_MPPT * 2;
        else
        {
//-------------------------------------------------------------------------------------------
          if (Ppv > old_Ppv)
          {
            if (Vpv > old_Vpv) {
              PV_PWM_DC = old_PV_PWM_DC - PV_PWM_INC_MPPT;
              PV_PWM_INC_ARRAY[0] = -1;
            }
            else {
              PV_PWM_DC = old_PV_PWM_DC + PV_PWM_INC_MPPT;
              PV_PWM_INC_ARRAY[0] = 1;
            }
          }
          else
          {
            if (Vpv > old_Vpv) {
              PV_PWM_DC = old_PV_PWM_DC + PV_PWM_INC_MPPT;
              PV_PWM_INC_ARRAY[0] = 1;
            }
            else {
              PV_PWM_DC = old_PV_PWM_DC - PV_PWM_INC_MPPT;
              PV_PWM_INC_ARRAY[0] = -1;
            }
          }
//-------------------------------------------------------------------------------------------
        }

        old_Vpv = Vpv; //save old voltage value
        old_Ipv = Ipv; //save old current value

        if (PV_PWM_DC > PV_PWM_MAX)          // check limits of PWM duty cyle and set to PWM_MAX
          PV_PWM_DC = PV_PWM_MAX;
        else if (PV_PWM_DC < PV_PWM_MIN)        // if pwm is less than PWM_MIN then set it to PWM_MIN
          PV_PWM_DC = PV_PWM_MIN;

        updatePWM( 1, PV_PWM_DC);

        //save old value
        old_PV_PWM_DC = PV_PWM_DC;

//adaptative method -------------------------------------------------------------------------

        PV_PWM_INC_ARRAY[5] = PV_PWM_INC_ARRAY[4];
        PV_PWM_INC_ARRAY[4] = PV_PWM_INC_ARRAY[3];
        PV_PWM_INC_ARRAY[3] = PV_PWM_INC_ARRAY[2];
        PV_PWM_INC_ARRAY[2] = PV_PWM_INC_ARRAY[1];
        PV_PWM_INC_ARRAY[1] = PV_PWM_INC_ARRAY[0];

        int arr_a[] = {1, 1, -1, -1, 1, 1};

        if ( array_cmp(PV_PWM_INC_ARRAY, arr_a, 6, 6))
        {
          PV_PWM_INC_MPPT = PV_PWM_INC_MPPT * 0.5;
          if (PV_PWM_INC_MPPT < PV_PWM_INC_MPPT_MIN)
            PV_PWM_INC_MPPT = PV_PWM_INC_MPPT_MIN;
        }

        int arr_b[] = {1, 1, 1, 1, 1, 1};
        int arr_c[] = { -1, -1, -1, -1, -1, -1};
        if ( array_cmp(PV_PWM_INC_ARRAY, arr_b, 6, 6) || array_cmp(PV_PWM_INC_ARRAY, arr_c, 6, 6))
        {
          //PV_PWM_INC_MPPT = PV_PWM_INC_MPPT_INITIAL;
          PV_PWM_INC_MPPT = PV_PWM_INC_MPPT * 2;
          if (PV_PWM_INC_MPPT > PV_PWM_INC_MPPT_INITIAL)
            PV_PWM_INC_MPPT = PV_PWM_INC_MPPT_INITIAL;
        }
        //serialSendExtra(PV_PWM_DC, PV_PWM_INC_MPPT, 0);
        //int PV_PWM_DC_TOSEND = PV_PWM_DC * 100 / PWM_MAX;
        //byte sendingByte2[6] = {syncbyte, 10, (byte)PV_PWM_DC_TOSEND, (byte)(Ipv/10), (byte)(old_Ipv/10), (byte)PV_PWM_INC_MPPT};
        //Serial.write(sendingByte2,6);

      }

//RPPT --------------------------------------------------------------------------------------
      else if (PV_mode == 2) { //RPPT
        //P_PV_ref

        int PV_PWM_DC;
        float Ppv = Ipv * Vpv;

        if (Ppv < P_PV_min)
          PV_PWM_DC = old_PV_PWM_DC + PV_PWM_INC_RPPT * 2;
        else
        {
//-------------------------------------------------------------------------------------------
          //if ( ((Ppv < P_PV_ref) && (Ppv > old_Ppv) && ( Vpv <= old_Vpv || Ipv >= old_Ipv)) || ((Ppv < P_PV_ref) && (Ppv < old_Ppv) && ( Vpv >= old_Vpv || Ipv <= old_Ipv)  )))
          if ( ((Ppv < P_PV_ref) && (Ppv > old_Ppv) && (Vpv >= V_mpp_t)) || ((Ppv < P_PV_ref) && (Ppv < old_Ppv) && (Vpv >= V_mpp_t)))
          {
            PV_PWM_DC = old_PV_PWM_DC + PV_PWM_INC_RPPT;
            PV_PWM_INC_POSITIVE_DIRECTION = PV_PWM_INC_POSITIVE_DIRECTION + 1;
            PV_PWM_INC_NEGATIVE_DIRECTION = 0;
          }
          else
          {
            PV_PWM_DC = old_PV_PWM_DC - PV_PWM_INC_RPPT;
            PV_PWM_INC_NEGATIVE_DIRECTION = PV_PWM_INC_NEGATIVE_DIRECTION + 1;
            PV_PWM_INC_POSITIVE_DIRECTION = 0;
          }
        }

        //saturation
        if (PV_PWM_DC > PV_PWM_MAX)          // check limits of PWM duty cyle and set to PWM_MAX
          PV_PWM_DC = PV_PWM_MAX;
        else if (PV_PWM_DC < PV_PWM_MIN)        // if pwm is less than PWM_MIN then set it to PWM_MIN
          PV_PWM_DC = PV_PWM_MIN;

        updatePWM( 1, PV_PWM_DC); //update dutycycle

//adaptative method -------------------------------------------------------------------------

        if ((old_Ppv < P_PV_ref) && (Ppv > P_PV_ref) && (old_Vpv > Vpv))
          PV_P_REF_PASS = PV_P_REF_PASS + 1;
        if ((old_Ppv > P_PV_ref) && (Ppv < P_PV_ref) && (old_Vpv < Vpv))
          PV_P_REF_PASS = PV_P_REF_PASS + 1;

        //increment reduction
        if ( PV_P_REF_PASS == 3 && PV_PWM_INC_RPPT > 1) {
          PV_P_REF_PASS = 0;
          PV_PWM_INC_RPPT = PV_PWM_INC_RPPT * 0.6;
          if (PV_PWM_INC_RPPT < PV_PWM_INC_RPPT_MIN)
            PV_PWM_INC_RPPT = PV_PWM_INC_RPPT_MIN;
        }

        //reset
        if (PV_PWM_INC_POSITIVE_DIRECTION == 5 || PV_PWM_INC_NEGATIVE_DIRECTION == 5)
        {
          PV_PWM_INC_POSITIVE_DIRECTION = 0;
          PV_PWM_INC_NEGATIVE_DIRECTION = 0;
          PV_PWM_INC_RPPT = PV_PWM_INC_RPPT * 1.6; //PV_PWM_INC_RPPT_INITIAL;
          if (PV_PWM_INC_RPPT > PV_PWM_INC_RPPT_INITIAL)
            PV_PWM_INC_RPPT = PV_PWM_INC_RPPT_INITIAL;
        }

        // save old values
        old_PV_PWM_DC = PV_PWM_DC;
        old_Ppv = Ppv;
        old_Vpv = Vpv;
        old_Ipv = Ipv;
      }

    }
//testing task duration
    duration2 = micros() - currentMicros;
    previousMicros2 = currentMicros;
  }

//--------------------------------------------------------------------------------
// TASK SOC
//--------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//OCV METHOD

  //IF IN OCV, BATTERY VOLTAGE SHOULD BE 2.1V/CELL (12.6V)
  //13.65V IS THE NORMAL VOLTAGE WHEN IN CHARGE
  if (currentMicros - previousMicros3 > interval_3) {

    //TEST THE BATTERY VOLTAGE WHEN RESTING TO CHECK BATTERY VOLTAGE
    if (BAT_connected == false && Vbat > 5) { //Vbat > 5V para garantir que as baterias estão ligadas
      bat_resting--;
      if (bat_resting == 0)
      {
        //limits
        //  V    V8bat  soc
        //12,8  102,4 100
        //12,6  100,8 75
        //12,3  98,4  50
        //12    96    25
        //11,8  94,4  0

        if(Vbat_f < 96.2)
          SOC_P = 20;
        else if(Vbat_f >= 96.2 && Vbat_f < 97.9)
          SOC_P = 17.852 * Vbat_f - 1697.9;
        else if(Vbat_f >= 97.9 && Vbat_f < 99.9)
          SOC_P = 15.241 * Vbat_f - 1442.6;
        else if(Vbat_f >= 99.9)
          SOC_P = 80;
          
        SOC_0 = SOC_P * Cmax / 100;
        bat_resting = ESS_resting_time;
      }
    }
    else
      bat_resting = ESS_resting_time;//intervalo de 10 segundos, bateria descansar 30 min

/////////////////////////////////////////////////////////////////////////////////////////////
//COULOMB METHOD

    if (BAT_connected) {
      //SOC calculation by method 1
      long SOC_SR = 360;

      SOC = (SOC_0 - (Ibat_f / SOC_SR));
      SOC_P = SOC * 100 / Cmax;
      SOC_0 = SOC;

      if (SOC_P > 100)
        SOC_0 = Cmax;
      if (SOC_P < 0)
        SOC_0 = 0;
    }

//    //acerta o  Pbat_ref todos os 10 sec
//    if (BAT_connected == true && BAT_mode == 2) {
//      float P_bat = Ibat_f * Vbat_f;
//      float P_bat_error = P_bat / Pbat_ref;
//      if(P_bat_error <= 0.9 || P_bat_error <= 1.1){
//        
//        Ibat_initial = Ibat_ref;
//        Ibat_ref = Pbat_ref / Vbat_f;
//        if (Ibat_ref > I_bat_ref_max)
//          Ibat_ref = I_bat_ref_max;
//        if (Ibat_ref < I_bat_ref_min)
//          Ibat_ref = I_bat_ref_min;
//
//        ramp_mode = true;
//        ramp_conf = true;
//        t_execution = 0;
//      }
//    }

    previousMicros3 = currentMicros;
  }
//-------------------------------------------------------------------------------------------
//TASK Communication
//to send data throught serial port only (for example, error messages, data
//-------------------------------------------------------------------------------------------
//for comunication
  if (currentMicros - previousMicros4 > interval_4) {
    //
    serialReceive(); //check if there are some data in serial port buffer

    //send measurement data
    float P_bat = Ibat_f * Vbat_f;
    float P_pv = Ipv * Vpv_f;

    if (BAT_connected)
      serialSendBat((float) SOC_P, (float)P_bat, (float)Vbat_f); //(SOC_P,P_bat, U_bat)
    else
      serialSendBat((float) SOC_P, 0, (float)Vbat_f); //(SOC_P,P_bat, U_bat)

    if (PV_connected || PV_disconnecting)
      serialSendPv((float)P_pv, (float)Vpv_f);
    else
      serialSendPv(0, (float)Vpv_f);

    if (DC_connected == true)
      serialSendLoad(true);
    else
      serialSendLoad(false);

    //LED error
    if (led_blinking)
      if (led_state == 1) {
        TURN_OFF_LED;
        led_state = 0;
      }
      else {
        TURN_ON_LED;
        led_state = 1;
      }

    //acerta o  Pbat_ref todos os 10 sec
    if (BAT_connected == true && BAT_mode == 2) {
      float P_bat_error = P_bat / Pbat_ref;
      if(P_bat_error <= 0.9 || P_bat_error <= 1.1){
        
        Ibat_initial = Ibat_ref;
        Ibat_ref = Pbat_ref / Vbat_f;
        if (Ibat_ref > I_bat_ref_max)
          Ibat_ref = I_bat_ref_max;
        if (Ibat_ref < I_bat_ref_min)
          Ibat_ref = I_bat_ref_min;

        ramp_mode = true;
        ramp_conf = true;
        t_execution = 0;
      }
    }
    
    //testing task duration
    //duration4 = micros() - currentMicros;
    previousMicros4 = currentMicros;
  }

}
