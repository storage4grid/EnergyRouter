/*--------------------------------------------------------------------------------------------------------
  S4G PROJECT_ENERGY ROUTER, UNIVERSIDADE NOVA DE LISBOA
  Authors: Nuno Vilhena, Carlos Roncero-Clemente, Vasco Gomes, Fabio Januario and Joao Martins
  This code is for an Arduino DUE with the aim of controlling DC/DC converters for battery and PV system.

  05/09/2018 v1.00
  new: started a new project from older code.
  15/10/2018 v3.00
  new: Add communication functions with RaspberryPI.
  16/10/2018 v3.10
  new: Modifications to allow control of converters through RaspberryPI.
  22/10/2018 v3.20
  new: Some modifications in ramp functions, control, add igbt interrupst and other bug fixes
  26/10/2018 v3.30
  new: Bug fixes in communication functions. Fix a bug that deny battery converter starts working 
        controlling DC link voltage if it receive a BAToff command first.
        Implement a current loop control for PV converter to "simulate" a PV system with a power source.
  30/10/2018 v3.40
  new: Changed limits protection functionality.
  30/10/2018 v3.50
  new: Set the system to work at nominal power
  12/11/2018  v4.00
  new: Add communication function BlackStart
  25/11/2018  v4.10
  new: Add fault and reset function to the LOAD DRIVER
       Optimazing PI controler for battery.
       Other optimizations.
       SOC.
  27/11/2018  v4.20
  new: PV control code improved. Add a ramp function to PV control when it is turned off.
  --/12/2018  v4.30
  new:
  --/12/2018  v4.40
  new:
  06/03/2019  v4.50
  new: Changed comunication functions in order to send Upv and Ubat.
  17/05/2019 v4.60
  new: included a condition that only allows the SOC calculation if the batteries are connected to the system.
  09/07/2019 v4.70
  new: now, when a error occurs, a extra msg is sent with the ER status in that moment.
  09/07/2019 v4.80
  new: Added a condition in "functions" to only allow the bat converter to turn on if the SOC is ok for that.
       Added a round function for that condition, if the limit is 80%, and the value is 79.9, it will round
       to 80% to impose a margin for the SOC value.
  03/10/2019 v4.90
  new: Bug corrections: add a 100ms delay in order to avoid false flag limits warnings.
       Changed the way the battery setpoint is defined. Now the Pbat_ref is redefined every 10 second, in the SOC task.
       Add a linear equation to estimate the SOC value when battery is resting at least during 10 minutes.
      
  -------------------------------------------------------------------------------------------------------*/
/*Specifications:
  xxxx
  xxxx
  */
