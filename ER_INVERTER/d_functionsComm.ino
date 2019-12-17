/* --------------------------------------------------------------------------------------
 * Communication functions
 * --------------------------------------------------------------------------------------
 * Copyright (c) 2019 Nuno Vilhena <nuv@uninova.pt>
 *
 * This file is part of ER Inverter.
 *
 * ER Inverter is free software; you can redistribute it and/or
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
 * License along with ER Inverter. If not, see: <http://www.gnu.org/licenses/>.
 * --------------------------------------------------------------------------------------
 *
 * Description: Communication functions to enable the ER control.
 * 
-----------------------------------------------------------------------------------------**/

void serialSendExtra(float value1, float value2, float value3){
  static byte * output1 = (byte *) &value1;
  static byte * output2 = (byte *) &value2;
  static byte * output3 = (byte *) &value3;
  byte message[] = {BEGIN, (byte)14, SEND_MSG, EXTRA, output1[0], output1[1], output1[2], output1[3], output2[0], output2[1], output2[2], output2[3], output3[0], output3[1], output3[2], output3[3]};
  Serial.write(message,16);
}

void serialSendID() {
  byte message[] = {BEGIN, (byte)3, SEND_MSG, ID_ID, ID};
  Serial.write(message,5);
}

void serialSendINV(float p, float q, float uac, boolean grid){
  static byte * output = (byte *) &p;
  static byte * output2 = (byte *) &q;
  static byte * output3 = (byte *) &uac;
  byte grid2 = (byte)0;
  if (grid){
    grid2 = (byte)1;
  }
  byte message[] = {BEGIN, (byte)15, SEND_MSG, MEASINV, output[0], output[1], output[2], output[3], output2[0], output2[1], output2[2], output2[3], output3[0], output3[1], output3[2], output3[3], grid2};
  Serial.write(message,17);
}

void serialSendVdc(float vdc){
  static byte * output = (byte *) &vdc;
  byte message[] = {BEGIN, (byte)6, SEND_MSG, MEAS_VDC, output[0], output[1], output[2], output[3]};
  Serial.write(message,8);
}

void serialSendINV_Gcheck(boolean flag){
  byte flag2 = (byte)0;
  if (flag) {
    flag2 = (byte)1;
  }
  byte message[] = {BEGIN, (byte)3, SEND_MSG, GCS, flag2};
  Serial.write(message,5);  
}

void serialSendError(int error){
  byte message[] = {BEGIN, (byte)3, SEND_ERROR, INV_ERROR, (byte)error};
  Serial.write(message,5);
}

void serialSendConf(byte flag, byte value){
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
    while(!SerialUSB);
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
            #ifdef DEBUG
              SerialUSB.println("ID_ID"); //debug
            #endif
            serialSendID();
            comm = true;
            digitalWrite(ledPin,HIGH);           
            first = first+8;          
            delay(5000);
            return;            
          } else {
            #ifdef DEBUG
              SerialUSB.println("Not ID_ID"); //debug 
            #endif             
            first++;            
          }
        } else { //without bytes
          int j=0;
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
  }
  else if (count_ka == COUNT_KA){
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
          #ifdef DEBUG
            SerialUSB.println("ID_ID"); //debug
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
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == MODE)) { //new mode          
          #ifdef DEBUG
            SerialUSB.print("MODE: "); //debug
            SerialUSB.println(input[first+4]); //debug
          #endif 
          if (input[first+4] == (byte)0) {
            receiveINV_OFF();
          } else if (input[first+4] == (byte)1) {
            receiveINV_SBY();
          } else if (input[first+4] == (byte)2) {
            receiveINV_ISLAND();
          }
          serialSendConf(MODE_CONF,CONF_TRUE);
          first = first+5;               
        }        
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == P)) { //new P
          byte inputVal[4] = {input[first+4], input[first+5], input[first+6], input[first+7]};
          float inP = *((float*)(inputVal));
          receiveINV_P(inP);
          serialSendConf(P_CONF,CONF_TRUE);  
          first = first+8; 
          #ifdef DEBUG
            SerialUSB.print("P :"); //debug      
            SerialUSB.print(inP); //debug
          #endif                                  
        }
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == Q)) { //new Q
          byte inputVal[4] = {input[first+4], input[first+5], input[first+6], input[first+7]};
          float inQ = *((float*)(inputVal));
          receiveINV_Q(inQ);
          serialSendConf(Q_CONF,CONF_TRUE); 
          first = first+8; 
          #ifdef DEBUG
            SerialUSB.print("Q :"); //debug      
            SerialUSB.print(inQ); //debug
          #endif
        }
        else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == GCS)) {
          receiveINV_Gcheck();
          first = first+4;
          #ifdef DEBUG
            SerialUSB.println("Receive GridConState Request.");
          #endif                                          
        } else if ((input[first+2] == RECEIVE_MSG) && (input[first+3] == BS)) { //BlackStart
          receiveBS();
          #ifdef DEBUG
            SerialUSB.println("BlackStart"); //debug
          #endif
          serialSendConf(BS_CONF,CONF_TRUE);          
          first = first+4;                                                
        } else {
          first++;
          #ifdef DEBUG
            SerialUSB.println("Error MSG 1"); //debug 
          #endif
        }        
      } else if(nb > 6) {
        first++;
        #ifdef DEBUG
          SerialUSB.println("Error MSG 2"); //debug 
        #endif          
      } else { //without bytes
        int j=0;
        for (int i = first; i<=last; i++){
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
