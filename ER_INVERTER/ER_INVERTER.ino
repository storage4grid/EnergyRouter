/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
  S4G PROJECT_ENERGY ROUTER, UNIVERSIDADE NOVA DE LISBOA
  Authors: Nuno Vilhena, Carlos Roncero-Clemente, Vasco Gomes, Fabio Januario and Joao Martins
  This code is for an Arduino DUE with the aim of controlling a bidirectional single phase
  converter (two branches).

  05/09/2018 v2.00
  new: Started a new project from older code.
  15/10/2018 v3.00
  new: Add communication functions with RaspberryPI.
  16/10/2018 v3.10
  new: Modifications to allow control of converters through RaspberryPI.
  22/10/2018 v3.20
  new: Modifications in the ramp functions and bug fixes.
  26/10/2018 v3.30
  new: Bug fixes in communication functions.
  30/10/2018 v3.40
  new: Revised the saturation limits of PWM values. Implement AntiIsland detection and IslandMode.
  03/11/2018 v3.50
  new: Bug fixes and performance improvements in AntiIsland detection and Island Mode functionalities. Improve PI controlers. Changed sample rate to 260us. Correction in SOGI algorithm (antiIsland).
  03/11/2018 v3.60
  new: Feitas alterações à parte de modo ilha, de modo a tentar resincronizar apos a rede voltar.
  03/11/2018 v3.70
  new: Increase the voltages to the nominal voltage (220Vdc).
        Changed the function that check when grid sync and connect the inverter.
        Now, the relay is first activated and after x seconds, the inverter starts working, in order to stabilize the rectification voltage.
        Was deleted some code related with island mode implmented in 3.60.
  10/11/2018 v3.80
  new: implement the resyncronization from a external signal.
  12/11/2018 v4
  new: Add communication funtion BlackStart
  23/11/2018 v4.10
  new: Some improvements on blackstart function
  27/11/2018 v4.20
  new: Add island mode operation (for GRID OFF operating) with voltage control and current+voltage control (for blackstart only)
  28/11/2018 v4.30
  new: Nominal voltage
  06/03/2019  v4.40
  new: Changed comunication functions in order to send Uac.
  
  //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*Specifications:
  xxxx
  xxxx
  //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
