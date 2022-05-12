// Arduino headers
#include <EEPROM.h>
#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE false

// My Headers
#include "FSM.h"
FSM_c FSM;

void setup() {
  // put your setup code here, to run once:
  delay(2000);
  Serial.begin(9600);
  delay(100);
  FSM.Initialize();
  motors.Stop();
  randomSeed(14); //Set Constant Seed to get Constant Results
  //get initial error
  RL.last_ts_RL = millis();
  RL.last_ts_Graph = millis();
  RL.last_ts_TrainTest = millis();
}

void loop() {
  RL.follow_line();
}
