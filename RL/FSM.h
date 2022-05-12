// My Headers
#include "RL.h"
RL_c RL;

class FSM_c{
  public:
    FSM_c() {}
    
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Function to Initialize Everything on the Robot
    void Initialize(bool printSerial=false){
      Serial.print("INITIALIZE\n");
      motors.Initialize(printSerial);
      line_Sensors.Initialize(printSerial);
      interface.Initialize();
      delay(1000);
      controls.Calibrate();
      //Find_Line_And_Join();
    }
};
