//************************************************************************************************************************************************************************
// Main loop.
//************************************************************************************************************************************************************************
void loop() {

  unsigned long currentMicros = micros(); //initial time;

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TASK implement SOGI e modulation part
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //int duration1 = currentMicros - previousMicros;
  if (currentMicros - previousMicros >= interval) {

//read grid current from analogue port+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    GRID_CURRENT = analogRead(AMPS_INV);
    GRID_CURRENT = GRID_CURRENT - SHIFT_VOLTAGE; //retira o offset
    //Igrid = (float)GRID_CURRENT * 3.3 / 4095.0;  //check if this calue is the right one
    //Igrid2 = Igrid * INV_I_scale;
    //Igrid = -Igrid2; //invert
    Igrid = (float)GRID_CURRENT * -0.0133515;
    
    add_to_array(I_ARRAY,array_size, Igrid);//*NEW*
    
//read grid voltage from analogue ports++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    GRID_VOLTAGE = analogRead(VOLTS_INV);     
    GRID_VOLTAGE = GRID_VOLTAGE - SHIFT_VOLTAGE; //retira o offset
    //Vgrid = (float)GRID_VOLTAGE * 3.3 / 4095.0; //conversao ppois está a ser utilizado 12 bits que da um maximo de 4096 para 3.3V
    //Vgrid = Vgrid * INV_V_scale ;
    Vgrid = GRID_VOLTAGE * 0.205271;

    if(Vgrid_max < Vgrid)
      Vgrid_max = Vgrid;

    add_to_array(V_ARRAY,array_size, Vgrid);//*NEW*

    rms_rate++;
    if(rms_rate == 4){
      add_to_array(V_ARRAY_RMS,array_size2, Vgrid);
      rms_rate = 0;
    }

//read DCLINK voltage from analogue ports++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      DCLINK_VOLTAGE = analogRead(VOLTS_DCLINK);
      //Vdclink = (float)DCLINK_VOLTAGE * 3.3 / 4095.0; //conversao ppois está a ser utilizado 12 bits que da um maximo de 4096 para 3.3V
      //Vdclink = Vdclink * DC_V_scale;
      Vdclink = DCLINK_VOLTAGE * 0.085421;//0.062662;
      
      //Filtering DClink voltage
      Vdc_aux = Vdclink*k1_fff - Vdc_aux_old*k2_fff;
      Vdc_f = Vdc_aux + Vdc_aux_old;
      Vdc_aux_old = Vdc_aux;
    
//running SOGI algorithm - returns SOGI_angle++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      SOGI_angle = SOGI1.Update(Vgrid);   

     
//connect the  converter+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(INV_connecting  && block_all== false){//if the inverter is sync with grid and the flag inverter_connecting is true
       if(INVERTER_mode == DCLINK_CTRL){
       
        if((int)SOGI1.Return_Wt() > W_max && (int)SOGI1.Return_Wt() < W_min){ //não está sincrono
            //RESET VARIABLES
            if(INV_reset == 0){
              SOGI1.Reset_SOGI();
              INV_reset =  5000000 / interval;
            }
        }
        else
        {
          //INV_connect_();
          INV_connecting = false;
          TURN_ON_INV_RELAY;
          //*****************************************
          t_execution_DCLINK = 0.0;
          time_mode_DCLINK = true;
          INV_reset = 0;
        }
        INV_reset--;        
      }
      else//island mode
        {
          INV_connect_Island_();
        }
  }

  //TIME COUNTING for stabilize DC rectifier voltage+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      if(time_mode_DCLINK){
        if( t_execution_DCLINK < time_limit_DCLINK)
          t_execution_DCLINK = t_execution_DCLINK + SAMPLERATE;
          
        else
        {
          time_mode_DCLINK = false;
          INV_connect_();   
        }
      }
        
  if(INV_disconnecting)//if the inverter is sync with grid and the flag inverter_connecting is true
       INV_disconnect_();

//check grid avaliable++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if( SOGI1.Return_Vd() <= 10 && (INVERTER_mode == DCLINK_CTRL || INV_connected == false))
      grid_avaliable = false;
    else if( SOGI1.Return_Vd() >= 10 && (INVERTER_mode == DCLINK_CTRL || INV_connected == false))
      grid_avaliable = true;

//INVERTER CONNECTED+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(INV_connected){
          
//check FLAGS++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if(INVERTER_mode == DCLINK_CTRL){
      if( SOGI1.Return_Wt() < W_min || SOGI1.Return_Wt() > W_max){// || SOGI1.Return_Vd() <= 10){
        Error_Treatment1();
        grid_avaliable =  false;
        serialSendExtra((float)SOGI1.Return_Vd(), SOGI1.Return_Wt(), 0);
        inv_ctrl = false;
      }
      else if(Vdc_f > Vdclink_max || (Vdc_f < Vdclink_min && time_mode == false) ){
        
        n_limits_error++;
        if(n_limits_error > 5000 || Vdc_f >= Vdclink_max2){ //2000 = 0.5sec
          Error_Treatment2();
          inv_ctrl = false;
          serialSendExtra((float)Vdclink, SOGI1.Return_Wt(), SOGI1.Return_Vd());
          n_limits_error = 0;
        }
      }
       else if(SOGI1.Return_Vd() > Vd_max || SOGI1.Return_Vd() < Vd_min  || abs(GRID_CURRENT) > Iinv_max){
        
        //n_limits_error++;
        //if(n_limits_error > 160){
          Error_Treatment2();
          inv_ctrl = false;
          serialSendExtra((float)Vdclink, SOGI1.Return_Wt(), SOGI1.Return_Vd());
          n_limits_error = 0;
        //}
      }
      else
        n_limits_error = 0;
    }
    else 
    if(INVERTER_mode == BLACKSTART || INVERTER_mode == ISLAND){
     if(Vdc_f > Vdclink_max  || Vdc_f < Vdclink_min ){

        n_limits_error++;
        if(n_limits_error > 5000 || Vdc_f >= Vdclink_max2){
          Error_Treatment2();
          inv_ctrl = false;
          serialSendExtra((float)Vdc_f, 0, 0);
        n_limits_error = 0;
        }
      }
      else if(abs(GRID_CURRENT) > Iinv_max || root_mean_square_voltage > Vinv_max ){
        //n_limits_error++;
        //if(n_limits_error > 5000 || Vdc_f >= Vdclink_max2){
          Error_Treatment2();
          inv_ctrl = false;
          serialSendExtra((float)root_mean_square_voltage, (float)GRID_CURRENT, (float)GRID_VOLTAGE);
        //n_limits_error = 0;
        //}
      }
      else if( (SOGI1.Return_Wt() < W_min || SOGI1.Return_Wt() > W_max) &&  time_mode == false && INVERTER_mode == BLACKSTART){

        grid_avaliable =  false;
        
        n_errors2++;
        if(n_errors2 == 5000){
          Error_Treatment5();
          n_errors2 = 0;
          grid_avaliable =  false;
          inv_ctrl = false;
          serialSendExtra((float)duration, 0, SOGI1.Return_Wt());
        }

      }
      else{
        n_limits_error = 0;
        n_errors2 = 0;
      }
  }

//INVERTER CONTROL CODE+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(inv_ctrl){

//RAMP MODE TO INCREASE DCLINK++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      if(ramp_mode){
          v_ref_ctrl = m_ramp * t_execution + v_dc_initial;   
        //increment 
        if( v_ref_ctrl < v_dc_ref)
          t_execution = t_execution + SAMPLERATE;
        else
        {
          ramp_mode = false;
          v_ref_ctrl = v_dc_ref;
        }
      }

//RAMP MODE TO INCREASE Q+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      if(ramp_mode2){
        if(Iq_given > Iq_initial){
          iq_ref = m_ramp2 * t_execution2 + Iq_initial;   
          //increment 
          if(iq_ref < Iq_given)
            t_execution2 = t_execution2 + SAMPLERATE;
          else
          {
            ramp_mode2 = false;
            iq_ref = Iq_given;         
          }
        }
        else{
          iq_ref = -m_ramp2 * t_execution2 + Iq_initial;   
          //increment 
          if(iq_ref > Iq_given)
            t_execution2 = t_execution2 + SAMPLERATE;
          else
          {
            ramp_mode2 = false;
            iq_ref = Iq_given;         
          }
          
        }   
      }
      
      //RAMP MODE TO INCREASE AC VOLTAGE++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      if(ramp_mode3){
          v_ref_ctrl = m_ramp * t_execution;   
        //increment 
        if( v_ref_ctrl < v_ac_ref_max)
          t_execution = t_execution + SAMPLERATE;
        else
        {
          ramp_mode3 = false;
          v_ref_ctrl = v_ac_ref_max;
        }
      }

//TIME COUNTING++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      if(time_mode){
        if( t_execution3 < time_limit)
          t_execution3 = t_execution3 + SAMPLERATE;
        else
          time_mode = false;
      }

//DC_LINK VOLTAGE CONTROL++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
if(INVERTER_mode == DCLINK_CTRL){   
     v_dc_error = v_ref_ctrl - Vdclink;  
    term_int = term_int - v_dc_error * SAMPLERATE * K_INT_DC;
    float term_prop = v_dc_error * K_PROP_DC;
    
    if (term_int > TERM_INT_MAX) 
      term_int = TERM_INT_MAX;    // using saturation function in order to speed up regulator response
    else if (term_int < TERM_INT_MIN) 
      term_int = TERM_INT_MIN;    
      
    id_ref = term_int - term_prop;

    // saturation of ID ref
    if (id_ref > id_ref_MAX) 
      id_ref = id_ref_MAX;    // using saturation function in order to speed up regulator response
    else if (id_ref < id_ref_MIN) 
      id_ref = id_ref_MIN;
      
//DEAD BEET CURRENT CONTROL++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    float Icos_ = id_ref * cos_fix_rad(SOGI_angle); //active current max = 20
    float Isin_ = iq_ref * sin_fix_rad(SOGI_angle); //reactive current max = 20
    float Iref = (Isin_ + Icos_);
   

    float Ierror = Iref - Igrid;  
    //duty_cycle      
    float DC = (1 + Vgrid/(2*Vdclink) + Igrid*Rfilter/(2*Vdclink) + Ierror*Lfilter/(SAMPLERATE*(2*Vdclink)))/2;
    
    //saturation between 0 and 1
    if (DC < DC_min)
        DC = DC_min;
    else if (DC > DC_max)
        DC = DC_max;

    //converting between 0 and PWM_MAX
    DC = DC * PWM_MAX; //DC = 1 -> 4095  

    //the first value is sent to the TOP of IGBT module 1 and the second value is sent to the BOTTON of IGBT module 2
    update_SyncPWM( (int)DC,  (int)DC);
}
//MODE ISLAND OR BLACKSTART
else
{
    //time variable
    t_execution_ac = t_execution_ac + SAMPLERATE;//duration_IM/1000000;
    if(t_execution_ac >= 0.020)
      t_execution_ac = 0.0;

    //SOGI_angle = 2*PI*50*time
    SOGI_angle_IM = 314.159 * t_execution_ac;

//BLACK START MODE++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(INVERTER_mode == BLACKSTART){
  
    //checking SOGI != simulatedSOGI
    if( abs(SOGI_angle_IM - SOGI_angle) > 3  && time_mode == false){
      peak_detection++;
      if(peak_detection > 5){
        grid_avaliable = true;
        led_blinking = true;
        peak_detection = 0;
        disableInterrupt(SYNC_PIN);
        }
    }
    else
      peak_detection = 0;
      
    if(grid_avaliable != true)
      SOGI_angle = SOGI_angle_IM;
    
    //Vrms calculation
    long sum_squared_voltage = 0;
    long squared_voltage = 0;
      
    for (int n=0; n < array_size2; n++)
    {
      squared_voltage = (int)V_ARRAY_RMS[n] * (int)V_ARRAY_RMS[n];
      sum_squared_voltage += squared_voltage;
    }
    long mean_square_voltage = sum_squared_voltage / array_size2;
    root_mean_square_voltage = sqrt32(mean_square_voltage);
    
    v_ac_error = v_ac_ref - root_mean_square_voltage; //v_ref_ctrl - v_ac_max;
    
    term_int_ac = term_int_ac + v_ac_error * SAMPLERATE * K_INT_AC;
    term_prop_ac = v_ac_error * K_PROP_AC;
    
    if (term_int_ac > TERM_INT_MAX_2) 
      term_int_ac = TERM_INT_MAX_2;    // using saturation function in order to speed up regulator response
    else if (term_int_ac < TERM_INT_MIN_2) 
      term_int_ac = TERM_INT_MIN_2;    
      
    id_ref = term_int_ac + term_prop_ac;
      
      // saturation of ID ref
    if (id_ref > id_ref_MAX_2) 
      id_ref = id_ref_MAX_2;    // using saturation function in order to speed up regulator response
    else if (id_ref < 0) 
      id_ref = 0;
      
//DEAD BEET CURRENT CONTROL++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    float Icos_ = id_ref * cos_fix_rad(SOGI_angle); //active current max = 20
    float Isin_ = iq_ref * sin_fix_rad(SOGI_angle); //reactive current max = 20
    float Iref = (Isin_ + Icos_);
   

    float Ierror = Iref - Igrid;  
    //duty_cycle      
    float DC = (1 + Vgrid/(2*Vdclink) + Igrid*Rfilter/(2*Vdclink) + Ierror*Lfilter/(SAMPLERATE*(2*Vdclink)))/2;
    
    //saturation between 0 and 1
    if (DC < DC_min)
        DC = DC_min;
    else if (DC > DC_max)
        DC = DC_max;

    //converting between 0 and PWM_MAX
    DC = DC * PWM_MAX; //DC = 1 -> 4095  

    //the first value is sent to the TOP of IGBT module 1 and the second value is sent to the BOTTON of IGBT module 2
    update_SyncPWM( (int)DC,  (int)DC);
  }
  else
  //ISLAND MODE++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(INVERTER_mode == ISLAND){

    float m_index = v_ac_ref_max / (2 * Vdclink);
    float Vcos_ = 1.15 * m_index * cos_fix_rad(SOGI_angle_IM); //active current max = 20
    float DC = Vcos_ * PWM_MAX * 0.5 + PWM_MAX * 0.5;
        
        //saturation between 0 and 1
    if (DC < PWM_INF)
        DC = PWM_INF;
    else if (DC > PWM_SUP)
        DC = PWM_SUP;
////  ////convert theta to the DAC port 
//        int theta_to_DAC = Vcos_*2047/200 +2047; //para ter um valor em percentagem
//        pinMode(DAC0, OUTPUT); // make sure to do it over and over again
//        analogWrite(DAC0, theta_to_DAC);
////        int theta_to_DAC_2 = Ibat*2047/10 +2047; //para ter um valor em percentagem
////        pinMode(DAC1, OUTPUT); // make sure to do it over and over again
////        analogWrite(DAC1, theta_to_DAC);
//////////convert theta to the DAC port

    //the first value is sent to the TOP of IGBT module 1 and the second value is sent to the BOTTON of IGBT module 2
    update_SyncPWM( (int)DC,  (int)DC);
  }
}



//calculating powers+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    Pmeas = (Igrid * Vgrid + I_ARRAY[array_size-1] * V_ARRAY[array_size-1])/2;
    Qmeas = (Igrid * V_ARRAY[array_size-1] - I_ARRAY[array_size-1] * Vgrid)/2;

    
    //filtered
    Pmeas_aux = Pmeas*k1_f - Pmeas_aux_old*k2_f;
    Pmeas_f = Pmeas_aux + Pmeas_aux_old;
    Pmeas_aux_old = Pmeas_aux;

    Qmeas_aux = Qmeas*k1_f - Qmeas_aux_old*k2_f;
    Qmeas_f = Qmeas_aux + Qmeas_aux_old;
    Qmeas_aux_old = Qmeas_aux; 
    
  }
  }
    //if(duration1 > 205)
    //  duration = duration1;
      
    duration = micros() - currentMicros;
    previousMicros = currentMicros;
  }
  

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//TASK Communication
//to send data throught serial port only (for example, error messages, data
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if (currentMicros - previousMicros2 > interval_2) {

    serialReceive(); //check if there are some data in serial port buffer
      
    //send measurement data
    if(INV_connected  == true){
      serialSendINV(Pmeas_f, -Qmeas_f, Vgrid_max, grid_avaliable);
      serialSendVdc(Vdc_f);
      Vgrid_max = 0.0; //reset variable
    }
    else
    {
      serialSendINV(0, 0, Vgrid_max, grid_avaliable);
      serialSendVdc(Vdclink);
      Vgrid_max = 0.0; //reset variable
    }  

      //LED error
      if(led_blinking)
        if(led_state == 1){
          TURN_OFF_LED;
          led_state = 0;
        }
        else{
          TURN_ON_LED;
          led_state = 1;
        }

    //serialSendExtra((float)root_mean_square_voltage, (float)duration, (float)id_ref);
    //serialSendExtra((float)Vdclink, (float)duration, (float)root_mean_square_voltage);
     //led_blinking = false;      
     //testing task duration
    duration2 = micros() - currentMicros;
    previousMicros2 = currentMicros;

    }
  }
