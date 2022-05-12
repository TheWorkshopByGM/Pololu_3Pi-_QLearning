// My Headers
#include "motors.h"
#include "line_Sensors.h"

// My Classes
Motors_c motors;
Line_Sensors_c line_Sensors;

// My Variables
# define Fwd_Speed_Bias Fwd_Speed //Forward Speed Bias (How fast the 3Pi+ moves forward)
//# define Kp 1.5*Fwd_Speed_Bias //Proportional Gain

class Control_c{
  public:
    Control_c() {}
    int Calibration_Rotation_Speed = Fwd_Speed;
    unsigned long calibration_start_ts = 0;
  
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Function that Calibrates min and max of all 5 line sensors
    void Calibrate(){
      Serial.print("CALIBRATE\n");
      interface.Wait_For_Button_Press(Push_Button_A_Pin);
      calibration_start_ts = millis();
      bool directionChange = false;
      
      line_Sensors.Calibrate();
      //motors.Run(-Calibration_Rotation_Speed,Calibration_Rotation_Speed,false);
      motors.Run(-25,25);
      while((millis()-calibration_start_ts)<5333){
        line_Sensors.Calibrate();
        delay(5);
      }
      motors.Stop();
      delay(50);
    }
  
  
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Function that Follows the Line and goes over Hard Corners
   /* void Follow_Line(){
      int Error = line_Sensors.GetError();
      if(line_Sensors.Line_Is_Straight){interface.Led_On(Yellow_Led_Pin);}
      else{interface.Led_Off(Yellow_Led_Pin);}
      if(line_Sensors.Robot_On_Line){interface.Led_On(Green_Led_Pin);}
      else{interface.Led_Off(Green_Led_Pin);}
      if(line_Sensors.Tight_Corner){interface.Led_On(Red_Led_Pin);}
      else{interface.Led_Off(Red_Led_Pin);}
      int Left_Speed = map(Error, -100, 100, Kp, -Kp) + Fwd_Speed_Bias;
      int Right_Speed = map(Error, -100, 100, -Kp, Kp) + Fwd_Speed_Bias;
      motors.Run(Left_Speed,Right_Speed);
    }*/
    
};
