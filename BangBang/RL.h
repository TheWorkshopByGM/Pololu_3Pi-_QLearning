#include "control.h"
Control_c controls;

class RL_c{
  public:
  RL_c() {}

  //Errors
  int current_error = 0;
  int prev_error = 0;

  int fwd_Bias = 30;

  unsigned int accumulated_error = 0;
  byte accumulator_counter = 0;
  int oscillation_accumulator = 0;
  
  unsigned long last_ts_RL = 0;
  unsigned long last_ts_Graph = 0;
  unsigned long last_ts_TrainTest = 0;

  //delays
  int RL_delay = 20;
  int Graph_update_delay_test = 50;
  unsigned long testing_duration = 2*60000;
  
  # define accum_size 330
  byte accum_array[accum_size] = {0};
  byte accum_array_osc[accum_size] = {0};
  int accum_array_index = 0;

  //Time Steps
  unsigned int ellapsed_time_RL = 0;
  unsigned int ellapsed_time_Graph = 0;
  unsigned long ellapsed_time_TrainTest = 0;

  void follow_line(){
    ellapsed_time_RL = millis()-last_ts_RL;
    ellapsed_time_Graph = millis()-last_ts_Graph;
    ellapsed_time_TrainTest = millis()-last_ts_TrainTest;

    if(ellapsed_time_RL>RL_delay){
      BangBang_control();
      last_ts_RL = millis();
    }
    if(ellapsed_time_Graph>Graph_update_delay_test){
      updateGraph();
      last_ts_Graph = millis();
    }
    if(ellapsed_time_TrainTest>testing_duration){ // Testing sequence done
      motors.Stop();
      interface.Wait_For_Button_Press(Push_Button_A_Pin);
      printGraph();
      interface.Wait_For_Button_Press(Push_Button_A_Pin);
      accum_array_index = 0;
      for(int i=0; i<accum_size; i++){
        accum_array[i] = 0;
        accum_array_osc[i] = 0;
      }
      last_ts_TrainTest = millis();
    }
    else if(endOfTrack()){
      motors.Stop();
      interface.Wait_For_Button_Press(Push_Button_A_Pin);
      printGraph();
      interface.Wait_For_Button_Press(Push_Button_A_Pin);
      accum_array_index = 0;
      for(int i=0; i<accum_size; i++){
        accum_array[i] = 0;
        accum_array_osc[i] = 0;
      }
      last_ts_TrainTest = millis();
    }
  }

  void BangBang_control(){
    line_Sensors.GetError();
    current_error = line_Sensors.Error;
    
    //Oscillations & Error Accumulators
    if((current_error<0) != (prev_error<0)){oscillation_accumulator++;}
    accumulated_error += abs(current_error);    
    accumulator_counter++; 
    prev_error = current_error;

    //PID
    int BangBang_Speed = 15;
    if(current_error>0){
      motors.Run(fwd_Bias-BangBang_Speed,fwd_Bias+BangBang_Speed);
    }
    else{
      motors.Run(fwd_Bias+BangBang_Speed,fwd_Bias-BangBang_Speed);
    }
  }

  bool endOfTrack(){
    line_Sensors.ReadSensors();
    if(line_Sensors.Sensors_values_Norm[0]>49){
      if(line_Sensors.Sensors_values_Norm[1]>49){
        if(line_Sensors.Sensors_values_Norm[2]>49){
          if(line_Sensors.Sensors_values_Norm[3]>49){
            if(line_Sensors.Sensors_values_Norm[4]>49){
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //DISPLAY FUNCTIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  void updateGraph(){
    if(accum_array_index<accum_size){
      accum_array[accum_array_index] = accumulated_error/accumulator_counter;
      accumulated_error = 0;
      accumulator_counter = 0;
      accum_array_osc[accum_array_index] = oscillation_accumulator; //(oscillation_accumulator*100)/osc_accumulator_counter;
      oscillation_accumulator = 0;
      accum_array_index++;
    }
  }

  void printGraph(){
    for(int i=0; i<accum_size-1; i++){
      Serial.print(accum_array[i]);
      Serial.print(",");
      Serial.print(accum_array_osc[i]);
      Serial.print("\n");
      Serial.print((accum_array[i]+accum_array[i+1])/2);
      Serial.print(",");
      Serial.print((accum_array_osc[i]+accum_array_osc[i+1])/2);
      Serial.print("\n");
    }
    Serial.print(accum_array[accum_size-1]);
    Serial.print(",");
    Serial.print(accum_array_osc[accum_size-1]);
    Serial.print("\n");
    Serial.print("\n");
    Serial.print("\n");
    delay(1);
    Serial.print("Time Ellapsed for Circuit Completion: ,");
    Serial.print(ellapsed_time_TrainTest);
    Serial.print("\n");
  } 
};
