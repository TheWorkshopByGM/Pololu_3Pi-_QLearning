#include "control.h"
Control_c controls;

class RL_c{
  public:
  RL_c() {}

  //RL Variables
  #define number_of_states 25
  #define number_of_actions 8
  int State_Action_Matrix[number_of_states][number_of_actions]= {0};
  int current_state = 0;
  int next_state = 0;
  int old_error, next_error;
  int Exp_Choice, Action_Choice;

  float epsilon = 100.0; //Will slowly decade to start exploiting instead of exploring
  float epsilon_decay = 0.999; //rate at which epsilon decreases
  float alpha = 0.04; //Learning Rate
  float gamma = 0.99;   
  
  int overshoot_punishment = 10;
  int error_multiplier = 10;
  double reward = 0;

  int fwd_Bias = 20;

  unsigned int accumulated_error = 0;
  byte error_accumulator_counter = 0;
  int oscillation_accumulator = 0;
  byte osc_accumulator_counter = 0;

  unsigned long last_ts_RL = 0;
  unsigned long last_ts_Graph = 0;
  unsigned long last_ts_TrainTest = 0;

  //delays
  int RL_delay = 20;
  int Graph_update_delay_train = 400;
  int Graph_update_delay_test = 50;
  unsigned long training_duration = 2*60000;
  unsigned long testing_duration = 2*60000;
  
  # define accum_size 330
  byte accum_array[accum_size] = {0};
  byte accum_array_epsilon[accum_size] = {0};
  byte accum_array_osc[accum_size] = {0};
  int accum_array_index = 0;

  //Train & Test
  bool training = true;
  bool lap_detected = false;

  //Time Steps
  unsigned int ellapsed_time_RL = 0;
  unsigned int ellapsed_time_Graph = 0;
  unsigned long ellapsed_time_TrainTest = 0;

  void follow_line(){
    ellapsed_time_RL = millis()-last_ts_RL;
    ellapsed_time_Graph = millis()-last_ts_Graph;
    ellapsed_time_TrainTest = millis()-last_ts_TrainTest;
    
    // Training
    if(training){
      if(ellapsed_time_RL>RL_delay){
        update_matrix();
        take_action();
        last_ts_RL = millis();
      }
      if(ellapsed_time_Graph>Graph_update_delay_train){
        updateGraph();
        last_ts_Graph = millis();
      }
      if(ellapsed_time_TrainTest>training_duration){ // Training sequence done
        motors.Stop();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        printGraph();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        printMatrix();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        printMatrixBinary();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        training = false;
        accum_array_index = 0;
        for(int i=0; i<accum_size; i++){
          accum_array[i] = 0;
          accum_array_epsilon[i] = 0;
          accum_array_osc[i] = 0;
        }
        last_ts_TrainTest = millis();
      }
      if(endOfTrack() && !lap_detected){
        lap_detected = true;
        accum_array[accum_array_index] = 255;
        accum_array_epsilon[accum_array_index] = epsilon;
        accum_array_osc[accum_array_index] = 255; //(oscillation_accumulator*100)/osc_accumulator_counter;
        accum_array_index++;
        motors.Run(fwd_Bias,fwd_Bias);
        delay(350);
      }
      else if(lap_detected){lap_detected = false;}
    }

    // Testing
    else{
      if(ellapsed_time_RL>RL_delay){
        dont_update_matrix();
        exploit_action(); 
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
        printMatrix();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        printMatrixBinary();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        training = false;
        accum_array_index = 0;
        for(int i=0; i<accum_size; i++){
          accum_array[i] = 0;
          accum_array_epsilon[i] = 0;
          accum_array_osc[i] = 0;
        }
        last_ts_TrainTest = millis();
      }
      else if(endOfTrack()){
        motors.Stop();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        printGraph();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        printMatrix();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        printMatrixBinary();
        interface.Wait_For_Button_Press(Push_Button_A_Pin);
        training = false;
        accum_array_index = 0;
        for(int i=0; i<accum_size; i++){
          accum_array[i] = 0;
          accum_array_epsilon[i] = 0;
          accum_array_osc[i] = 0;
        }
        last_ts_TrainTest = millis();
      }
    }
  }

  void exploit(){
    float best_action = State_Action_Matrix[current_state][0];
    for(int i = 1; i<number_of_actions; i++){
      if(State_Action_Matrix[current_state][i] > best_action){
        best_action = State_Action_Matrix[current_state][i];
        Action_Choice = i;
      }
    }
  }

  void explore(){
    Action_Choice = (int) random(1,number_of_actions);
  }

  void getReward(){
    if(abs(next_error) < 10){reward = 1000 - next_error*next_error*overshoot_punishment*error_change_sign();}
    else{reward = -next_error*next_error - next_error*next_error*overshoot_punishment*error_change_sign();}
    if(line_Sensors.Line_Is_Straight){reward += 10000;}
  }

  int error_change_sign(){
    osc_accumulator_counter++;
    if((next_error<0) == (old_error<0)){return 0;}
    else{oscillation_accumulator++;return 1;}
  }

  void update_matrix(){
    line_Sensors.GetError();
    next_error = line_Sensors.Error;
    accumulated_error += abs(next_error); 
    error_accumulator_counter++;    
    next_state = getCurrentState(line_Sensors.Error);
    //Select Reward
    getReward();
    //Find Max_q
    double max_q = State_Action_Matrix[next_state][0];
    for(int i = 1; i<number_of_actions; i++){
      if(State_Action_Matrix[next_state][i]> max_q){
        max_q = State_Action_Matrix[next_state][i];
      }
    }
    max_q = max_q/10.0;
    //Update Matrix
    float q = State_Action_Matrix[current_state][Action_Choice]/10.0;
    double q_new = (1-alpha)*q+alpha*(reward+gamma*max_q);
    if(q_new*10<-32768){
      State_Action_Matrix[current_state][Action_Choice] = -32768;
    }
    else if(q_new*10>32767){
      State_Action_Matrix[current_state][Action_Choice] = 32767;
    }
    else{
      State_Action_Matrix[current_state][Action_Choice] = q_new*10;
    }
  }

  void dont_update_matrix(){
    line_Sensors.GetError();
    next_error = line_Sensors.Error;
    accumulated_error += abs(next_error);    
    error_accumulator_counter++; 
    next_state = getCurrentState(line_Sensors.Error);
  }

  int getCurrentState(int Error){
    int new_state = current_state;
    new_state = current_state%5;
    new_state = new_state*5;
    if(Error < 10 && Error > -10){new_state += 0;}
    else if(Error < 20 && Error > -20){new_state += 1;}
    else if(Error < 30 && Error > -30){new_state += 2;}
    else if(Error < 50 && Error > -50){new_state += 3;}
    else{new_state += 4;}
    return new_state;
  }

  void action(int Action_Choice, int Error){
    int mySpeed[] = {0,2,4,6,8,10,15,20};
    int thisSpeed = mySpeed[Action_Choice] + (current_state%5) * 5;
    if(Error<0){
      motors.Run(fwd_Bias+thisSpeed,fwd_Bias-thisSpeed);
    }
    else{
      motors.Run(fwd_Bias-thisSpeed,fwd_Bias+thisSpeed);
    }
  }

  void take_action(){
    old_error = next_error;
    current_state = next_state;
    Exp_Choice = random(0,100);
    if(Exp_Choice>epsilon){exploit();}
    else{explore();}
    epsilon = epsilon*epsilon_decay;
    if(current_state == 0 || current_state == 1){
      action(0,line_Sensors.Error);
    }
    else{
      action(Action_Choice,line_Sensors.Error); 
    }
  }

  void exploit_action(){
    old_error = next_error;
    current_state = next_state;
    exploit();
    if(current_state == 0 || current_state == 1){
      action(0,line_Sensors.Error);
    }
    else{
      action(Action_Choice,line_Sensors.Error); 
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
      accum_array[accum_array_index] = accumulated_error/error_accumulator_counter;
      accumulated_error = 0;
      error_accumulator_counter = 0;
      accum_array_epsilon[accum_array_index] = epsilon;
      accum_array_osc[accum_array_index] = oscillation_accumulator; //(oscillation_accumulator*100)/osc_accumulator_counter;
      oscillation_accumulator = 0;
      osc_accumulator_counter = 0;
      accum_array_index++;
    }
  }

  void printGraph(){
    for(int i=0; i<accum_size-1; i++){
      Serial.print(accum_array[i]);
      Serial.print(",");
      Serial.print(accum_array_epsilon[i]);
      Serial.print(",");
      Serial.print(accum_array_osc[i]);
      Serial.print("\n");
      delay(10);
      Serial.print((accum_array[i]+accum_array[i+1])/2);
      Serial.print(",");
      Serial.print((accum_array_epsilon[i]+accum_array_epsilon[i+1])/2);
      Serial.print(",");
      Serial.print((accum_array_osc[i]+accum_array_osc[i+1])/2);
      Serial.print("\n");
      delay(10);
    }
    Serial.print(accum_array[accum_size-1]);
    Serial.print(",");
    Serial.print(accum_array_epsilon[accum_size-1]);
    Serial.print(",");
    Serial.print(accum_array_osc[accum_size-1]);
    Serial.print("\n");
    Serial.print("\n");
    Serial.print("\n");
    delay(10);
    Serial.print("Time Ellapsed for Circuit Completion: ,");
    Serial.print(ellapsed_time_TrainTest);
    Serial.print("\n");
  } 

  void printMatrix(){
    for(int i=0; i<number_of_states; i++){
      for(int j=0; j<number_of_actions; j++){
        Serial.print(State_Action_Matrix[i][j]);
        Serial.print(",");
      }
      Serial.print("\n");
      delay(5);
    }
  }

  void printMatrixBinary(){
    for(int i=0; i<number_of_states; i++){
      float myMax = -2147483648;
      for(int j=0; j<number_of_actions; j++){
        if(State_Action_Matrix[i][j]>myMax){
          myMax = State_Action_Matrix[i][j];
        }
      } 
      for(int j=0; j<number_of_actions; j++){
        if(State_Action_Matrix[i][j]==myMax){Serial.print("1,");}
        else{Serial.print("0,");}
      }
      Serial.print("\n");
      delay(5);
    }
  }
};
