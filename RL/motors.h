// PIN assigment
# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

// Defining Motor Directions
# define REV HIGH
# define FWD LOW

// Defining Speed Limits
# define Max_Speed 150 //set to 125 usually
# define Min_Speed 15
# define Fwd_Speed 30

class Motors_c {
  public: 
    Motors_c() {}

    void Initialize(bool printSerial=false){
      pinMode(L_PWM_PIN,OUTPUT);
      pinMode(L_DIR_PIN,OUTPUT);
      pinMode(R_PWM_PIN,OUTPUT);
      pinMode(R_DIR_PIN,OUTPUT);
    }

    void Run( int leftSpeed, int rightSpeed, bool printSerial=false){
      bool leftDir, rightDir;
    
      //Setting DIRECTIONS
      if(leftSpeed<0){leftDir = REV;} else{leftDir = FWD;}
      if(rightSpeed<0){rightDir = REV;} else{rightDir = FWD;}
    
      //Setting SPEEDS
      leftSpeed = abs(leftSpeed);
      rightSpeed = abs(rightSpeed);
      //if leftSpeed above Max
      if(leftSpeed >= rightSpeed && leftSpeed > Max_Speed){ 
        rightSpeed = map(rightSpeed, 0, leftSpeed, 0, Max_Speed);
        leftSpeed = Max_Speed;
      }
      //if rightSpeed above Max
      else if(rightSpeed > leftSpeed && rightSpeed > Max_Speed){ 
        leftSpeed = map(leftSpeed, 0, rightSpeed, 0, Max_Speed);
        rightSpeed = Max_Speed;
      }
      //if leftSpeed below Min
      if(leftSpeed < Min_Speed/2){leftSpeed = 0;} 
      else if(leftSpeed < Min_Speed){leftSpeed = Min_Speed;}
      //if rightSpeed below Min
      if(rightSpeed < Min_Speed/2){rightSpeed = 0;} 
      else if(rightSpeed < Min_Speed){rightSpeed = Min_Speed;}
      
      //Send Values to Actuators
      digitalWrite(L_DIR_PIN,leftDir);
      digitalWrite(R_DIR_PIN,rightDir);
      analogWrite( L_PWM_PIN, leftSpeed );
      analogWrite( R_PWM_PIN, rightSpeed );
    }

    void Stop(bool printSerial=false){ 
      Run(0,0);
    }
};
