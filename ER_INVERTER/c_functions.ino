 //************************************************************************************************************************************************************************
// Functions
//************************************************************************************************************************************************************************

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//SOGI ALGORITHM
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
class SOGI
{
  private:
    // Class Member Variables
    //constants
    static constexpr float SOGI_W0 = 314.159;
    static constexpr float _2PI = 6.283;
    static constexpr float _2PI_4 = 1.5708;

    //controlors
    float SOGI_GAIN;// = 0.6;
    float SOGI_Kp;// = 0.4;
    //static const float SOGI_Ti = 0.08;
    float SOGI_Ki;// = 5.0;//SOGI_Kp/SOGI_Ti;

    //variables locais
    float SOGI_sin = 0.0;
    float SOGI_cos = 0.0;
    float SOGI_Vd = 0.0;
    float SOGI_Vq = 0.0;
    float SOGI_delta_W = 0.0;
    float SOGI_Park[4];
    //variaveis globais
    float SOGI_alpha = 0.0;
    float SOGI_beta = 0.0;
    float SOGI_theta = 0.0;
    float SOGI_W = 0.0;
    float SOGI_angle = 0.0;
    float SOGI_Vq_EI = 0.0;
    float SOGI_var[5];

    float SOGI_theta_AI = 0.0;
    float cf = 0.0;

    float SOGI_SAMPLERATE;// = 0.000200;

    // Constructor - creates a SOGI
    // and initializes the member variables and state
  public:
    SOGI(float SAMPLERATE, float GAIN, float Kp, float Ki) {
      SOGI_SAMPLERATE = SAMPLERATE;
      SOGI_GAIN = GAIN;
      SOGI_Kp = Kp;
      SOGI_Ki = Ki;
    };

    float Update(float x)
    {
      //ALPHA BETA TRANSFORMATION
      SOGI_var[0] = x - SOGI_alpha;
      SOGI_var[1] = SOGI_var[0] * SOGI_GAIN;
      SOGI_var[2] = SOGI_var[1] - SOGI_beta;
      SOGI_var[3] = SOGI_var[2] * SOGI_W;
      SOGI_alpha = SOGI_alpha + (SOGI_SAMPLERATE * SOGI_var[3]); //Alpha component of the grid voltage-first integrator
      SOGI_var[4] = SOGI_var[4] + (SOGI_SAMPLERATE * SOGI_alpha); //second integrator
      SOGI_beta = SOGI_var[4] * SOGI_W; //Beta component of the grid voltage
      //Vd & Vq CALCULATION
      SOGI_sin = sin_fix_rad(SOGI_theta);//angle in radians
      SOGI_cos = cos_fix_rad(SOGI_theta);//angle in radian
      SOGI_Park[0] = SOGI_alpha * SOGI_cos;
      SOGI_Park[1] = SOGI_beta * SOGI_sin;
      SOGI_Park[2] = (-SOGI_alpha) * SOGI_sin;
      SOGI_Park[3] = SOGI_beta * SOGI_cos;
      SOGI_Vd = SOGI_Park[0] + SOGI_Park[1]; //d component of the grid voltage
      SOGI_Vq = SOGI_Park[2] + SOGI_Park[3]; //q component of the grid voltage

      //----------SOGI PI WITH BACKWARD EULER INTEGRATOR-------------------------------------------
      SOGI_Vq_EI = SOGI_Vq_EI + SOGI_SAMPLERATE * SOGI_Ki * SOGI_Vq;
      SOGI_delta_W = SOGI_Vq_EI + SOGI_Kp * SOGI_Vq;
      //-------------------------------------------------------------------------------------------
      SOGI_W = SOGI_W0 + SOGI_delta_W;
      SOGI_theta = SOGI_theta + SOGI_SAMPLERATE * SOGI_W;

      //saturation SOGI teta
      if (SOGI_theta > _2PI)
        SOGI_theta = SOGI_theta - _2PI; //theta angle will saturate at 2PI radians


      //antiisland detection=======================================================================
      cf = (SOGI_W0 - SOGI_W)*0.016 + 0.1;
      SOGI_theta_AI = SOGI_theta*(1 + cf);
      //saturation SOGI teta
      if (SOGI_theta_AI > _2PI)
        SOGI_theta_AI = _2PI; //theta angle will saturate at 2PI radians
      if (SOGI_theta_AI < 0)
        SOGI_theta_AI = 0; //theta angle will saturate at 2PI radians
        
      SOGI_angle = SOGI_theta;// + _2PI_4; //_2PI / 4; //+0.04*_2PI;   add 90º
      return SOGI_angle;
    };

    float Return_Vd()
    {
      return SOGI_Vd;
    };

    float Return_Wt()
    {
      return (SOGI_W);
    };

    void Reset_SOGI()
    {
    SOGI_sin = 0.0;
    SOGI_cos = 0.0;
    SOGI_Vd = 0.0;
    SOGI_Vq = 0.0;
    SOGI_delta_W = 0.0;
    //variaveis globais
    SOGI_alpha = 0.0;
    SOGI_beta = 0.0;
    SOGI_theta = 0.0;
    SOGI_W = 0.0;
    SOGI_angle = 0.0;
    SOGI_Vq_EI = 0.0;
    };

    float cos_fix_rad(float x)
    {
      static const int max_in = (1UL << 16);
      static const int max_out = (1UL << 14);
      static const float _2PI = 6.283;

      int aux = cos_fix(x * max_in / _2PI);

      return (float)aux / max_out;
    };

    float sin_fix_rad(float x)
    {
      static const int max_in = (1UL << 16);
      static const int max_out = (1UL << 14);
      static const float _2PI = 6.283;

      int aux = sin_fix(x * max_in / _2PI);

      return (float)aux / max_out;
    };
};
//SOGI(float SAMPLERATE, float GAIN, float Kp, float Ki){
SOGI SOGI1(SAMPLERATE, 0.6, 0.4, 5.0);

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This routine treats the IGBT errors (received from ERROR_OUT of IGBTs
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  void Error_Treatment_INV(){
//    Serial.println("ERROR in BAT driver");
    //put led blinking
    delayMicroseconds(10);
    if(digitalRead(Error_IGBT1) || digitalRead(Error_IGBT2)){   
      shutdown_();
      serialSendError(3);
    }
    //send a error msg to master in order to block the drivers
  }

  void Error_Treatment2(){
//      Serial.println("Out of Ranges");
      INV_disconnect_();
      serialSendError(2);
  }

  void Error_Treatment1(){
//      Serial.println("grid off");
      INV_disconnect_();
      serialSendError(1);
  }

  void Error_Treatment5(){
//      Serial.println("grid off");
      INV_disconnect_();
      serialSendError(5);
  }

  void SYNC_FUNCTION(){
    
    long sync_time = micros();
    long duration = sync_time - sync_old_time;
    
    if(duration > 18000 && duration < 22000){ //a period is 20ms = 20000 us
      if(INVERTER_mode == BLACKSTART){
        if(t_execution_ac > 0.0005){ //5% of a period of 20ms
          t_execution_ac = t_execution_ac + 0.0005;//duration_IM/1000000;
//        if(t_execution_ac < 0.0)
//          t_execution_ac = 0.0;
        }
        else{
          t_execution_ac = 0.0;
        }
 
      //LED blinking each period
        if(led_state == 1){
          TURN_OFF_LED;
          led_state = 0;
        }
        else{
          TURN_ON_LED;
          led_state = 1;
        }
      }
      else
        INVERTER_mode = BLACKSTART; //if the grid return signal starter and it is in island mode, change to blacktart mode
    }
    
    sync_old_time = sync_time;
  }
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This routine shutdown the system
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  void shutdown_(void){
 
    stop_pwm();
    INV_disconnect_();

    //TURN_ON_LED;
    led_blinking = true;
    
    //sending error code to RBpi

    //block arduino until reset
    block_all =  true;
  }
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This routine disconnect the INVERTER
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  void INV_disconnect_(void){ 
    stop_pwm();
    //delayMicroseconds(1000);
    TURN_OFF_INV_RELAY;

    //reset  variables
    id_ref = 0;
    iq_ref = 0;

    led_blinking = false;
    TURN_ON_LED;

    if(INVERTER_mode == BLACKSTART){
      disableInterrupt(SYNC_PIN);
    }

    INV_connected = false;
    INV_disconnecting = false;
  }
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This routine connect the inverter to the grid, if grid avaliable, GRID ON MODE, DCLINK CONTROLLING
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  void INV_connect_(void){
   
    //reset  variables
    id_ref = 0;
    iq_ref = 0;
    v_ref_ctrl = 0;
    
    //variables PI
    term_int = 0;

    //TURN_ON_INV_RELAY;
    //delay(20);
    //read the rectification voltage
    DCLINK_VOLTAGE = analogRead(VOLTS_DCLINK);
    Vdclink = (float)DCLINK_VOLTAGE * 3.3 / 4095.0; //conversao ppois está a ser utilizado 12 bits que da um maximo de 4096 para 3.3V
    Vdclink = Vdclink * DC_V_scale;
    
    //turn on ramp mode
      //ramp*************************************
      v_dc_initial = Vdclink; //sqrt32(2)*v_ac_ref/2; //;
      t_execution = 0.0;
      ramp_mode = true;
      //*****************************************
      t_execution3 = 0.0;
      time_mode = true;

      inv_ctrl =  true;
     
      led_blinking = false;
      TURN_OFF_LED;

      INV_connected = true;
      INV_connecting = false;

      start_pwm(0);

  }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This routine connect the inverter to the system in ISLAND MODE - for ISLAND AND BLACKSTART MODES
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  void INV_connect_Island_(void){
   
    //reset  variables
    id_ref = 0;
    iq_ref = 0;
    v_ref_ctrl = 0;
    
    SOGI1.Reset_SOGI();

    //for island mode*******
    t_execution_ac = 0.0;
    //variables PI
    term_int_ac = 0.0;
    term_int = 0.0;

    peak_detection = 0;
    //time mode
      //*****************************************
      t_execution3 = 0.0;
      time_mode = true;
      //*****************************************
    inv_ctrl =  true;

       //turn on ramp mode for ISLAND MODE
//    if(INVERTER_mode == ISLAND){
//      //ramp*************************************
//      t_execution = 0.0;
//      ramp_mode3 = true;
//      //*****************************************
//    }
    if(INVERTER_mode == BLACKSTART){
      enableInterrupt(SYNC_PIN, SYNC_FUNCTION, RISING);
    }
    
    led_blinking = false;
    TURN_OFF_LED;

    INV_connected = true;
    INV_connecting = false;

    TURN_ON_INV_RELAY;
    start_pwm(0);
  }
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Those routines calculate cos and sin 
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This routine add a element to an array
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  void add_to_array(float *array_, int len, float value){

   // shift elements of array n to 0          
   for ( int i = len-1; i > 0; i-- ) {
      array_[ i ] = array_[ i-1 ];
   }

   array_[ 0 ] = value;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This routine add a element to an array
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned int sqrt32(unsigned long n) 
{ 
unsigned int c = 0x8000; 
unsigned int g = 0x8000; 

for(;;) { 

     if(g*g > n) {
          g ^= c; 
     }
     
     c >>= 1; 

     if(c == 0) {
          return g; 
     }

     g |= c; 
    
     } 
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// COMMUNICAION FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

void receiveINV_OFF(){
  INV_disconnecting = true;
}

void receiveINV_SBY(){
    if(!INV_connected){
      INVERTER_mode = DCLINK_CTRL;
      INV_connecting = true;
    }
}

void receiveINV_P(float p){

}

void receiveINV_ISLAND(){
  if(INV_connected && INVERTER_mode == BLACKSTART){
    INVERTER_mode = ISLAND;
    SOGI1.Reset_SOGI();
    disableInterrupt(SYNC_PIN);
  }
  else     
    if(!INV_connected){
    INVERTER_mode = ISLAND;
    INV_connecting = true;
    }
}

void receiveINV_Q(float q){

//    INVERTER_mode = ISLAND;
//    INV_connecting = true;
    
  if(INV_connected && INVERTER_mode == DCLINK_CTRL){

      //float q_max = sqrt32(Inv_S*Inv_S - Pmeas_f*Pmeas_f);
      if( q > 1250)
        q = 1250;
      if(q < -750)
        q = -750;
        
      //iq_ref = -q / SOGI1.Return_Vd();
      Iq_given = -q / SOGI1.Return_Vd();
      ramp_mode2 = true;     
  }
}

void receiveINV_Gcheck(){
  serialSendINV(Vdclink, Pmeas_f, -Qmeas_f, grid_avaliable);
}

void receiveBS(){
  if(INV_connected && INVERTER_mode == ISLAND){
    INVERTER_mode = BLACKSTART;
    SOGI1.Reset_SOGI();
    enableInterrupt(SYNC_PIN, SYNC_FUNCTION, RISING);
  }
  else     
    if(!INV_connected){
    INVERTER_mode = BLACKSTART;
    INV_connecting = true;
    }
}

void commLost(){
    INV_disconnecting = true;
    led_blinking = true;
}
