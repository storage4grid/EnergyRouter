//*******************************************************************************************
//SETUP: This routine is automatically called at powerup/reset
//*******************************************************************************************
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
