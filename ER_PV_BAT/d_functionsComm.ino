void serialSendID() {
  byte message[] = {BEGIN, (byte)3, SEND_MSG, ID_ID, ID};
  Serial.write(message,5);   
}
void serialSendBat(float value1, float value2, float value3){ //(SoC,Pbat,Ubat)
  static byte * output1 = (byte *) &value1;
  static byte * output2 = (byte *) &value2;
  static byte * output3=(byte *) &value3;
  byte message[] = {BEGIN, (byte)14, SEND_MSG, BAT, output1[0], output1[1], output1[2], output1[3], output2[0], output2[1], output2[2], output2[3], output3[0], output3[1], output3[2], output3[3]};
  Serial.write(message,16);  
}
void serialSendPv(float value1, float value2){ //(Ppv,Upv)
  static byte * output1 = (byte *) &value1;
  static byte * output2 = (byte *) &value2;
  byte message[] = {BEGIN, (byte)10, SEND_MSG, PV, output1[0], output1[1], output1[2], output1[3], output2[0], output2[1], output2[2], output2[3]};
  Serial.write(message,12);  
}
void serialSendLoad(boolean value){
  byte value1 = (byte)0;
  if (value){
    value1 = (byte)1;
  }
  byte message[] = {BEGIN, (byte)3, SEND_MSG, LOADS, value1};
  Serial.write(message,5);
}
void serialSendBatError(int error){
  byte message[] = {BEGIN, (byte)3, SEND_ERROR, BAT_ERROR, (byte)error};
  Serial.write(message,5);
}

void serialSendPvError(int error){
  byte message[] = {BEGIN, (byte)3, SEND_ERROR, PV_ERROR, (byte)error};
  Serial.write(message,5);
}

void serialSendExtra(float value1, float value2, float value3){
  static byte * output1 = (byte *) &value1;
  static byte * output2 = (byte *) &value2;
  static byte * output3 = (byte *) &value3;
  byte message[] = {BEGIN, (byte)14, SEND_MSG, EXTRA, output1[0], output1[1], output1[2], output1[3], output2[0], output2[1], output2[2], output2[3], output3[0], output3[1], output3[2], output3[3]};
  Serial.write(message,16);
}

void serialSendConf(byte flag, byte value){ //(BAT_CONF,CONF_TRUE/CONF_FALSE) or (PV_CONF,CONF_TRUE/CONF_FALSE)
  byte message[] = {BEGIN, (byte)3, SEND_MSG, flag, value};
  Serial.write(message,5);  
}

void serialSendKA(){
  byte message[] = {BEGIN, (byte)2, SEND_MSG, KA};
  Serial.write(message,4);  
}

void commSetup(){
  #ifdef DEBUG
    SerialUSB.begin(9600); //debug
    while(!SerialUSB); //debug
  #endif
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,LOW);
  while(!Serial); //wait open serial port
  while(!comm) {
    delay(500);
    int nbytes = Serial.available();
    for (int i = 1; i<=nbytes; i++) { //read available bytes to buffer input
      input[last+i] = Serial.read();
    }
    last = last+nbytes;
    while (first<=last) {       
      if (input[first] == BEGIN) { //Begin of message
        #ifdef DEBUG
          SerialUSB.println("BEGIN"); //debug
        #endif
        int nb = (int)input[first+1]; //number of bytes 
        #ifdef DEBUG
          SerialUSB.print("Number Bytes: "); //debug
          SerialUSB.println(nb); //debug 
        #endif              
        if (last-(first+1) >= nb) {
          if (((input[first+2] == RECEIVE_MSG)||(input[first+2] == RECEIVE_MSG2)) && (input[first+3] == ID_ID)) { //ID request
            byte inputVal[4] = {input[first+4], input[first+5], input[first+6], input[first+7]};
            float inSoC = *((float*)(inputVal));
            receiveSoCfile(inSoC);
            #ifdef DEBUG
              SerialUSB.print("ID_ID: "); //debug
              SerialUSB.println(inSoC);
            #endif
            serialSendID();
            comm = true;
            digitalWrite(ledPin,HIGH);
            first = first+8;
            delay(5000);          
            return;            
          } else {
            first++;
            #ifdef DEBUG
              SerialUSB.println("Error MSG 1"); //debug
            #endif                        
          }
        } else if (nb > 6) {
          first++;
          #ifdef DEBUG
            SerialUSB.println("Error MSG 2"); //debug
          #endif          
        } else { //without bytes 
          int j = 0;
          for (int i = first; i<last; i++){
            input[j]=input[i];
            j++;
          }
          first = 0;
          last = j;
          #ifdef DEBUG
            SerialUSB.println("Without bytes"); //debug 
          #endif          
          break;  
        }    
      } else { //is not BEGIN byte
        first = first+1;
      }
    } //end of while
    first = 0;
    last = -1;
  }
}

void serialReceive() {  
  int nbytes = Serial.available();

  if (nbytes > 0){
    count_ka = 0;
    count_wm = 0;
    count_nm = 0;
  }
  else if (count_ka < COUNT_KA){
    count_ka++;
  } else if (count_ka == COUNT_KA){
    if (count_wm == 0){
      serialSendKA();
      count_wm++;
    } else if (count_wm < COUNT_WM){
      count_wm++;
    } else if (count_wm == COUNT_WM) {
      if (count_nm < COUNT_NM){
        count_wm = 0;
        count_nm++;
      } else {
        commLost();
        count_ka = 0;
        count_wm = 0;
        count_nm = 0;        
        #ifdef DEBUG
          SerialUSB.println("No Communication."); //debug
        #endif
      }       
    }
  }
  
  for (int i = 1; i<=nbytes; i++) { //read available bytes to buffer input
    input[last+i] = Serial.read();
  }
  last = last+nbytes;  
  while (first<=last) {        
    if (input[first] == BEGIN) { //Begin of message
      #ifdef DEBUG
        SerialUSB.println("BEGIN"); //debug
      #endif
      int nb = (int)input[first+1]; //number of bytes
      #ifdef DEBUG
        SerialUSB.print("Number Bytes: "); //debug
        SerialUSB.println(nb); //debug                        
      #endif
      if (last-(first+1) >= nb) {   
        if (((input[first+2] == RECEIVE_MSG)||(input[first+2] == RECEIVE_MSG2)) && (input[first+3] == ID_ID)) { //ID request
            byte inputVal[4] = {input[first+4], input[first+5], input[first+6], input[first+7]};
            float inSoC = *((float*)(inputVal));
            receiveSoCfile(inSoC);
            #ifdef DEBUG
              SerialUSB.print("ID_ID: "); //debug
              SerialUSB.println(inSoC);
            #endif
            serialSendID();
            comm = true;
            digitalWrite(ledPin,HIGH);
            first = first+8;                       
        }    
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == KA)) { //KA
          #ifdef DEBUG
            SerialUSB.println("KA"); //debug
          #endif          
          first = first+4;            
        }            
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == MODEBAT)) { //new mode         
          #ifdef DEBUG
            SerialUSB.print("MODEBAT: "); //debug
            SerialUSB.println(input[first+4]); //debug
          #endif       
          if (input[first+4] == (byte)0) {
            receiveBat_OFF();
          }
          else if (input[first+4] == (byte)1) {
            receiveBat_SBY();
          }
          serialSendConf(MODEBAT_CONF,CONF_TRUE); 
          first = first+5;       
        }      
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == MODEPV)) { //new mode         
          #ifdef DEBUG
            SerialUSB.print("MODEPV: "); //debug
            SerialUSB.println(input[first+4]); //debug
          #endif       
          if (input[first+4] == (byte)0) {
            receivePv_OFF();
          } else if (input[first+4] == (byte)1) {
            receivePv_MPPT();
          }
          serialSendConf(MODEPV_CONF,CONF_TRUE); 
          first = first+5;       
        }          
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == PV)) { //new Ppv         
          byte inputVal[4] = {input[first+4], input[first+5], input[first+6], input[first+7]};
          float inPpv = *((float*)(inputVal));
          receivePv_RPPT(inPpv);
          serialSendConf(PV_CONF,CONF_TRUE);
          first = first+8;  
          #ifdef DEBUG
            SerialUSB.print("PV :"); //debug      
            SerialUSB.println(inPpv); //debug
          #endif                  
        }
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == BAT)) { //new Ibat          
          byte inputVal[4] = {input[first+4], input[first+5], input[first+6], input[first+7]};
          float inPBat = *((float*)(inputVal));
          receiveBat_P(inPBat);
          serialSendConf(BAT_CONF,CONF_TRUE); 
          first = first+8;   
          #ifdef DEBUG
            SerialUSB.print("BAT: "); //debug      
            SerialUSB.println(inPBat); //debug
          #endif                          
        }
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == LOADS)) { //new Load          
          #ifdef DEBUG
            SerialUSB.print("LOADS: "); //debug
            SerialUSB.println(input[first+4]); //debug
          #endif       
          if (input[first+4] == (byte)0) {
            receiveLoad(false);
          } else if (input[first+4] == (byte)1) {
            receiveLoad(true);
          }
          serialSendConf(LOADS_CONF,CONF_TRUE); 
          first = first+5;                           
        } else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == BS)) { //BlackStart
          receiveBS();
          #ifdef DEBUG
            SerialUSB.println("BlackStart"); //debug
          #endif
          serialSendConf(BS_CONF,CONF_TRUE);          
          first = first+4;                                                
        } else { 
          first = first +1; 
          #ifdef DEBUG
            SerialUSB.println("Error MSG 1"); //debug
          #endif                              
        }        
      } else if (nb > 6) {
        first ++;
        #ifdef DEBUG
          SerialUSB.println("Error MSG 2"); //debug
        #endif        
      } else { //without bytes
        int j = 0;
        for (int i = first; i<last; i++){
          input[j]=input[i];
          j++;
        }
        first = 0;
        last = j;
        #ifdef DEBUG
          SerialUSB.println("Without bytes"); //debug
        #endif        
        return; 
      }     
    } else { //is not BEGIN byte
      first = first+1;
      #ifdef DEBUG
        SerialUSB.println("Not BEGIN"); //debug
      #endif      
    }
  } //end of while
  first = 0;
  last = -1;
}
