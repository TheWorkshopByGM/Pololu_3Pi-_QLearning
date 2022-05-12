// Pin Assignment
# define Yellow_Led_Pin 13
# define Red_Led_Pin 17 //WARNING: Shared with Push_Button_C_Pin !!!
# define Green_Led_Pin 30 //WARNING: Shared with Push_Button_B_Pin !!!

# define Push_Button_A_Pin 14
# define Push_Button_B_Pin 30 //WARNING: Shared with Green_Led_Pin !!!
# define Push_Button_C_Pin 17 //WARNING: Shared with Red_Led_Pin !!!

# define Buzzer_Pin 6

# define Buzz false

class Interface_c {
  public:
    Interface_c() {}

    void Led_On(int Led_Pin){
      pinMode(Led_Pin,OUTPUT);
      if(Led_Pin == Yellow_Led_Pin){digitalWrite(Yellow_Led_Pin, HIGH);}
      else{digitalWrite(Led_Pin, LOW);}
    }
    void Led_Off(int Led_Pin){
      pinMode(Led_Pin,OUTPUT);
      if(Led_Pin == Yellow_Led_Pin){digitalWrite(Yellow_Led_Pin, LOW);}
      else{digitalWrite(Led_Pin, HIGH);}
    }

    void Initialize(){
      pinMode(Buzzer_Pin, OUTPUT);
      pinMode(Yellow_Led_Pin, OUTPUT);
      pinMode(Red_Led_Pin, OUTPUT);  //WARNING: To use Push_Button_C_Pin set it to INPUT !!!
      pinMode(Green_Led_Pin, OUTPUT);  //WARNING: To use Push_Button_B_Pin set it to INPUT !!!
      pinMode(Push_Button_A_Pin, INPUT);
      for(int i = 0; i<3; i++){  //Initializing Routine
        Led_Off(Yellow_Led_Pin);
        Led_Off(Red_Led_Pin);
        Led_Off(Green_Led_Pin);
        digitalWrite(Buzzer_Pin, LOW);
        delay(300);
        Led_On(Yellow_Led_Pin);
        Led_On(Red_Led_Pin);
        Led_On(Green_Led_Pin);
        if(Buzz){analogWrite(Buzzer_Pin,100);}
        delay(300);
      }
      delay(500);
      Led_Off(Yellow_Led_Pin);
      Led_Off(Red_Led_Pin);
      Led_Off(Green_Led_Pin);
      digitalWrite(Buzzer_Pin, LOW);
    }
    
    void Buzz_The_Buzzer(int frequency, int duration_ms=1){
      if(duration_ms==0){}
      else{
        float delay_us = 1000000/(2*frequency);
        float repetitions = (duration_ms*1000000)/(2000*delay_us); 
        for(int i = 0; i<repetitions; i++){
          digitalWrite(Buzzer_Pin, HIGH);
          delayMicroseconds(delay_us);
          digitalWrite(Buzzer_Pin, LOW);
          delayMicroseconds(delay_us);
        }
      }
    }

    void Wait_For_Button_Press(int Button_Pin){
      pinMode(Button_Pin, INPUT);
      pinMode(Yellow_Led_Pin,OUTPUT);
      pinMode(Buzzer_Pin,OUTPUT);
      
      unsigned long last_ts;
      unsigned long ellapsed_ts;
      last_ts = millis();
      
      while(digitalRead(Button_Pin)!=0){
        ellapsed_ts = millis()-last_ts;
        if(ellapsed_ts > 2000){
          Led_On(Yellow_Led_Pin);
          if(Buzz){analogWrite(Buzzer_Pin,100);}
        }
        if(ellapsed_ts > 2200){
          last_ts = millis();
          Led_Off(Yellow_Led_Pin);
          digitalWrite(Buzzer_Pin, LOW);
        }
      }
      for(int i=0; i<3; i++){
        Led_On(Yellow_Led_Pin);
        if(Buzz){analogWrite(Buzzer_Pin,100);}
        delay(200);
        Led_Off(Yellow_Led_Pin);
        digitalWrite(Buzzer_Pin, LOW);
        delay(300);
      }
    }
};
