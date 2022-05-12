// My Headers
#include "interface.h"
Interface_c interface;

class Line_Sensors_c {
  public:
    //Info About Robot
    int Number_Of_Sensors = 5; // Number of sensors (Used to loop over them) 
    # define Black_Line_Threshold 50 // NORMNALIZED limit for which the sensor detects a line (ie: 50 = 50%)
    # define Emit_Pin 11
    int Line_Sensors_Pins[5] = {A11,A0,A2,A3,A4};
    float Tight_Corner_Error_Increment = 1.05;
  
    //Variables used for Reading the Sensors
    # define SensorReadingTimeout 3000 // Timeout Limit
    # define Update_Threshold 0.1 //10% // Sensor Limit Update Threshold
    int Sensors_max[5] = {0,0,0,0,0}; //Array to store the individual Max of Every Sensor
    int Sensors_min[5] = {10000,10000,10000,10000,10000}; //Array to store the individual Min of Every Sensor
    int Sensors_values[5] = {0,0,0,0,0}; //Array to store the individual Values of Every Sensor
    int Sensors_values_Norm[5] = {0,0,0,0,0}; //Array to store the Normalized individual Values of Every Sensor 
                                              //(Every Sensor is Mapped to a value between 0 and 100 depending on its min/max)
    int Sensors_Weights_Norm[5] = {0,0,0,0,0};//Array to store the Normalized Weights of All Sensors 
                                              //(All Sensor are Mapped so that their Sum is a value between 0 and 100)

    //Variables Used for Computing Error
    int Error = 0; //Intialization of the Error variable

    //Variables Used to Compute States
    # define Error_History_Size 30 //Size of Error History Array (Affects how long it takes for the robot to characterize the line as STRAIGHT)
    # define Straight_Line_Error_Threshold 20 //Error limit to consider line straight (The smaller the smaller the error needs to be)
    int Error_History[Error_History_Size] = {}; //Array used to store past errors and decide if line is straight
    int Error_History_Position = 0;

    # define Sensor_On_Line_History_Size 5 //Size of Error History Array (Affects how long it takes for the robot to characterize the line as STRAIGHT)
    int Sensor_On_Line_History[Sensor_On_Line_History_Size] = {}; //Array used to store past errors and decide if line is straight
    int Sensor_On_Line_History_Position = 0;

    //Robot State Checks
    bool Line_Is_Straight = false;
    bool Robot_On_Line = false;
    bool Tight_Corner = false;
  
    Line_Sensors_c() {}


    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Initialization Routine for Sensors
    void Initialize(bool printSerial=false){
      //Turn on IR LEDs
      pinMode(Emit_Pin, OUTPUT);
      digitalWrite(Emit_Pin, HIGH);
      //Set all sensors to be inputs
      for(int i = 0; i<Number_Of_Sensors; i++){
        pinMode(Line_Sensors_Pins[i], INPUT);
      }
      for(int i = 0; i<Error_History_Size; i++){
        Error_History[i] = 100; //Initialize Error_History to be "Not a straight line"
      }
      for(int i = 0; i<Sensor_On_Line_History_Size; i++){
        Sensor_On_Line_History[i] = 0; //Initialize Error_History to be "Not a straight line"
      }
    }
     

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Basic function that reads all the sensors value in parallel
    void readAllLineSensors(bool printSerial=false){
      //Charge Capacitor
      for(int i = 0; i<Number_Of_Sensors; i++){
        pinMode(Line_Sensors_Pins[i], OUTPUT);
      }
      for(int i = 0; i<Number_Of_Sensors; i++){
        digitalWrite(Line_Sensors_Pins[i], HIGH);
      }
      delay(10);
    
      //Read Delay
      for(int i = 0; i<Number_Of_Sensors; i++){
        pinMode(Line_Sensors_Pins[i], INPUT);
      }

      unsigned long start_time; // t_1
      unsigned long end_time[] = {0,0,0,0,0};   // t_2
      int sensors_done_counter = 0; //Counter to keep track of the number of sensors that are done reading
      
      start_time = micros();
      while( sensors_done_counter < Number_Of_Sensors  &&(micros() < (start_time+SensorReadingTimeout))) {
        for(int i=0; i<Number_Of_Sensors; i++){
          if(digitalRead(Line_Sensors_Pins[i]) == LOW && end_time[i]==0){
            end_time[i] = micros();
            sensors_done_counter++;
          }
        }
      }
      
      unsigned long elapsed_time[] = {0,0,0,0,0}; // t_elapsed
      for(int i=0; i<Number_Of_Sensors; i++){
        elapsed_time[i] = end_time[i] - start_time;
        Sensors_values[i] = elapsed_time[i];
      }
    }

    

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    //Function that normalizes readings and updates limits
    void NormalizeResults(bool printSerial=false){
      for(int i=0; i<Number_Of_Sensors; i++){
        if((Sensors_values[i] < Sensors_min[i]) && (Sensors_values[i] > ((1-Update_Threshold)*Sensors_min[i]) )){
          Sensors_min[i] = Sensors_values[i];
        }
        else if(Sensors_values[i] < Sensors_min[i]){
          Sensors_values[i] = Sensors_min[i];
        }
        if((Sensors_values[i] > Sensors_max[i]) && (Sensors_values[i] < ((1-Update_Threshold)*Sensors_max[i]) )){
          Sensors_max[i] = Sensors_values[i];
        }
        else if(Sensors_values[i] > Sensors_max[i]){
          Sensors_values[i] = Sensors_max[i];
        }
        Sensors_values_Norm[i] = map(Sensors_values[i], Sensors_min[i], Sensors_max[i], 0, 100);
      }
    }


    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Function that finds the maximum and minimum values of each sensor
    void Calibrate(bool printSerial=false){
      readAllLineSensors(printSerial);
      for(int i=0; i<Number_Of_Sensors; i++){
        if(Sensors_values[i]>Sensors_max[i]){
          Sensors_max[i] = Sensors_values[i];
        }
        if(Sensors_values[i]<Sensors_min[i]){ 
          Sensors_min[i] = Sensors_values[i];
        }
      }
    }
    

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Function that Checks for Robot States
    void Update_Robot_States(bool printSerial=false){
      //Check for Straight Line
      Line_Is_Straight = true;
      for(int i = 0; i<Error_History_Size; i++){
        if(abs(Error_History[i]) > Straight_Line_Error_Threshold){Line_Is_Straight = false;}
      }
      if(Line_Is_Straight){interface.Led_On(Yellow_Led_Pin);}
      else{interface.Led_Off(Yellow_Led_Pin);}

      //Check for Robot on Line
      Robot_On_Line = false;
      for(int i = 0; i<Number_Of_Sensors; i++){
        if(Sensors_values_Norm[i] > Black_Line_Threshold){Robot_On_Line = true;}
      }
      Sensor_On_Line_History[Sensor_On_Line_History_Position] = Robot_On_Line;
      if(Sensor_On_Line_History_Position==Sensor_On_Line_History_Size-1){
        Sensor_On_Line_History_Position = 0;
      }
      else{Sensor_On_Line_History_Position++;}
      int On_Line_History_Sum = 0;
      for(int i = 0; i<Sensor_On_Line_History_Size; i++){
        On_Line_History_Sum += Sensor_On_Line_History[i];
      }
      if(On_Line_History_Sum == 0){Robot_On_Line = false;interface.Led_Off(Green_Led_Pin);}
      else{Robot_On_Line = true;interface.Led_On(Green_Led_Pin);}
      
      //Check for Tight Corner
      if( (Sensors_values_Norm[0] > Black_Line_Threshold) || (Sensors_values_Norm[4] > Black_Line_Threshold)){
        Tight_Corner = true;
        interface.Led_On(Red_Led_Pin);
      }
      else if((Sensors_values_Norm[1] > Black_Line_Threshold) || (Sensors_values_Norm[2] > Black_Line_Threshold) || (Sensors_values_Norm[3] > Black_Line_Threshold)){
        Tight_Corner = false;
        interface.Led_Off(Red_Led_Pin);
      }
    }


    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Function that reads all line sensors, normalizes them and checks for robot states
    int ReadSensors(bool printSerial=false){
      readAllLineSensors();
      NormalizeResults();
      Update_Robot_States();
    }


    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Function that finds which direction the robot has to follow
    int GetError(bool printSerial=false){
      //Read, Normalize and update states
      ReadSensors();

      //Normalize Central Sensors Weights
      int TotalWeights = 0;
      for(int i = 0; i<Number_Of_Sensors; i++){ TotalWeights += Sensors_values_Norm[i]; } //Get Total Weight of Sensors
      TotalWeights += Sensors_values_Norm[0]+Sensors_Weights_Norm[4];
      if(TotalWeights == 0){ TotalWeights = 1; } //If all sensors = 0 eliminate div by 0 error
      for(int i = 0; i<Number_Of_Sensors; i++){
        Sensors_Weights_Norm[i] = map(Sensors_values_Norm[i],0,TotalWeights,0,100);
      }

      //Finding Error
      int Weight_Left = 2*Sensors_Weights_Norm[0] + Sensors_Weights_Norm[1] + 0.5*Sensors_Weights_Norm[2];
      int Weight_Right = 2*Sensors_Weights_Norm[4] + Sensors_Weights_Norm[3] + 0.5*Sensors_Weights_Norm[2];

      //Only update sensor if one sensor is on line
      //If line is lost keep same error as before
      if(Robot_On_Line){
        Error = Weight_Left - Weight_Right;
      }
      if(Tight_Corner && !Robot_On_Line){
        if(Error>0){Error = 100;}
        else if(Error < 0){Error = -100;}
      }

      //Update Error History
      if(Error_History_Position == Error_History_Size-1){ //If Pointer reached end of Array: update value and loop back
        Error_History[Error_History_Position] = Error;
        Error_History_Position = 0;
      }
      else{                                               //Else: Update and increment pointer
        Error_History[Error_History_Position] = Error; 
        Error_History_Position += 1;
      }
      return Error;
    }
};
