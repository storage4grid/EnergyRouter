//*******************************************************************************************
//This routine is automatically called at powerup/reset
//*******************************************************************************************
void setup() {

  
  //set resolutions for analogue ports
  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)
  analogReadResolution(12);   // set the analog input resolution to 12 bit

  //declare digital pins
  pinMode(Error_IGBT1, INPUT);
  pinMode(Error_IGBT2, INPUT);
  pinMode(Error_IGBT3, INPUT);

  pinMode(Reset_IGBT3, OUTPUT);
  digitalWrite(Reset_IGBT3, HIGH);
  
  pinMode(LED_, OUTPUT);
  pinMode(13, OUTPUT);//led da placa
  digitalWrite(13, LOW);

  //setup PWM pins
  //2 pwm pins are activated, without sync functionality and 15kHz
  PWM_MAX = setup_pwm(2 , 0, 15000);
  PWM_LIM_SUP = PWM_MAX * 0.95;
  PWM_LIM_INF = PWM_MAX * 0.05;
  
  disable_comp(1); //complementary will be disable in PV converter

  pinMode(DC_LOAD_S, OUTPUT);
  TURN_OFF_DC_SWITCH;

  //define limits for PWM dutycycle
  PV_PWM_MAX = PWM_MAX*0.9;                  // the value for pwm duty cyle max = 90%
  PV_PWM_MIN = PWM_MAX*0.02;                  // the value for pwm duty cyle min = 10%
  PV_PWM_START = PWM_MAX*0.05;                // the value for pwm duty cyle initial = half
  
  PV_PWM_INC_RPPT_INITIAL = PWM_MAX*0.05;  // 10%
  PV_PWM_INC_RPPT = PV_PWM_INC_RPPT_INITIAL;
  PV_PWM_INC_RPPT_MIN  = PWM_MAX*0.005; //0.5% o dutycycle

  PV_PWM_INC_MPPT_INITIAL = PWM_MAX*0.05; //5% o dutycycle
  PV_PWM_INC_MPPT = PV_PWM_INC_MPPT_INITIAL;
  PV_PWM_INC_MPPT_MIN = PWM_MAX*0.008; //0.5% o dutycycle

  old_PV_PWM_DC = PV_PWM_START;

  SOC_P = SOC_0*100/Cmax;

  // define an interrupt for input errors
  attachInterrupt(digitalPinToInterrupt(Error_IGBT1), Error_Treatment_PV, RISING);  // Interrupt attached to the pin Error_IGBT1
  attachInterrupt(digitalPinToInterrupt(Error_IGBT2), Error_Treatment_BAT, RISING);  // Interrupt attached to the pin Error_IGBT2
  //attachInterrupt(digitalPinToInterrupt(Error_IGBT3), Error_Treatment_DCLOAD, FALLING);  // Interrupt attached to the pin Error_IGBT3 - error is HIGH to LOW state

  commSetup(); //communications
    
  TURN_ON_LED; //is ready

}
