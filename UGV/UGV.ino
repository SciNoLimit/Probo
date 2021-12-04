#define left_motor_ENCA 18   // Defining right motor encoder A and encoder B
#define left_motor_ENCB 19   

#define right_motor_ENCA 20    // Defining right motor encoder A and encoder B 
#define right_motor_ENCB 21

#define right_motor_IN1 6     
#define right_motor_IN2 7

#define left_motor_IN1 4
#define left_motor_IN2 5

#define right_motor_pwmPin 3
#define left_motor_pwmPin 2

long distance = 1200; // Distance in mm

volatile long right_motor_pos = 0;
volatile long left_motor_pos = 0;
volatile long right_motor_posPrev = 0;
volatile long left_motor_posPrev = 0;

unsigned long previousT = 0;
// float left_eprev = 0;
// float right_eprev = 0;
// float left_eintegral = 0;
// float right_eintegral = 0;

float PID(float dt, float kp, float ki, float kd, long target, long pos, float eIntegral = 0 , float ePrev = 0 );

void setup() {
  delay(10000);
  pinMode(right_motor_ENCA, INPUT_PULLUP);
  pinMode(right_motor_ENCB, INPUT_PULLUP);
  
  pinMode(left_motor_ENCA, INPUT_PULLUP);
  pinMode(left_motor_ENCB, INPUT_PULLUP);
  
  pinMode(right_motor_IN1, OUTPUT);
  pinMode(right_motor_IN2, OUTPUT);
  
  pinMode(left_motor_IN1, OUTPUT);
  pinMode(left_motor_IN2, OUTPUT);
  
  pinMode(left_motor_pwmPin, OUTPUT);
  pinMode(right_motor_pwmPin, OUTPUT);

  Serial.begin(115200);
  
  attachInterrupt(digitalPinToInterrupt(left_motor_ENCA), left_motor_ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_motor_ENCB), left_motor_ISR_B, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(right_motor_ENCA), right_motor_ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_motor_ENCB), right_motor_ISR_B, CHANGE);

  Serial.print(right_motor_pos);
  Serial.print(" , ");
  Serial.println(left_motor_pos);
  
}

void loop() {

  long left_motor_targetPOS = -(3072*distance)/222.8;
  long right_motor_targetPOS = -(3076*distance)/222.8;

  float left_kp = 1;
  float left_ki = 0.0013;
  float left_kd = 0.0003;

  float right_kp = 1;
  float right_ki = 0.0013 ;
  float right_kd = 0.0003;

  unsigned long currentT = micros();
  float deltaT = ((currentT - previousT)/1.0e6); // deltaT in sec
  previousT = currentT;

float  right_u = PID(deltaT, right_kp, right_ki, right_kd, right_motor_targetPOS, right_motor_pos);
float  left_u = PID(deltaT, left_kp, left_ki, left_kd, left_motor_targetPOS, left_motor_pos);

  float right_pwr = fabs(right_u);
    if (right_pwr > 255) right_pwr = 255;
  float left_pwr = fabs(left_u);
    if (left_pwr > 255) left_pwr = 255;

//    if (right_pwr < 6) right_pwr = 0;
//    if (left_pwr < 6) left_pwr = 0;

  int right_dir = 1;
    if (right_u < 0 ) right_dir = -1;
  int left_dir = 1;
    if (left_u < 0 ) left_dir = -1; 

  
  setMotor(right_dir, right_pwr, right_motor_pwmPin, right_motor_IN1, right_motor_IN2);
  setMotor(left_dir, left_pwr, left_motor_pwmPin, left_motor_IN1, left_motor_IN2);
  
  
  
  if((right_motor_pos != right_motor_posPrev) || (left_motor_pos != left_motor_posPrev)){
    Serial.print(right_motor_pos);
    Serial.print(" , ");
    Serial.print(left_motor_targetPOS);
    Serial.print(" , ");
    Serial.println(left_motor_pos);
    
//    Serial.print(right_pwr);
//    Serial.print(" , ");
//    Serial.println(left_pwr);

    right_motor_posPrev =  right_motor_pos;
    left_motor_posPrev =  left_motor_pos; 
  }
 // Serial.println(deltaT, 6);  
}

 void right_motor_ISR_A(){  

  // look for a low-to-high on channel A
  if (digitalRead(right_motor_ENCA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(right_motor_ENCB) == LOW) {  
      right_motor_pos = right_motor_pos - 1;         // CW
    } 
    else {
      right_motor_pos = right_motor_pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right_motor_ENCB) == HIGH) {   
      right_motor_pos = right_motor_pos - 1;          // CW
    } 
    else {
      right_motor_pos = right_motor_pos + 1;          // CCW
    }
  }
 
}

void right_motor_ISR_B(){  

  // look for a low-to-high on channel A
  if (digitalRead(right_motor_ENCB) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(right_motor_ENCA) == HIGH) {  
      right_motor_pos = right_motor_pos - 1;         // CW
    } 
    else {
      right_motor_pos = right_motor_pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right_motor_ENCA) == LOW) {   
      right_motor_pos = right_motor_pos - 1;          // CW
    } 
    else {
      right_motor_pos = right_motor_pos + 1;          // CCW
    }
  }
 
}

void left_motor_ISR_A(){  

  // look for a low-to-high on channel A
  if (digitalRead(left_motor_ENCA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(left_motor_ENCB) == LOW) {  
      left_motor_pos = left_motor_pos + 1;         // CW
    } 
    else {
      left_motor_pos = left_motor_pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left_motor_ENCB) == HIGH) {   
      left_motor_pos = left_motor_pos + 1;          // CW
    } 
    else {
      left_motor_pos = left_motor_pos - 1;          // CCW
    }
  }
 
}

void left_motor_ISR_B(){  

  // look for a low-to-high on channel A
  if (digitalRead(left_motor_ENCB) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(left_motor_ENCA) == HIGH) {  
      left_motor_pos = left_motor_pos + 1;         // CW
    } 
    else {
      left_motor_pos = left_motor_pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left_motor_ENCA) == LOW) {   
      left_motor_pos = left_motor_pos + 1;          // CW
    } 
    else {
      left_motor_pos = left_motor_pos - 1;          // CCW
    }
  }
 
}
  
  void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
    analogWrite(pwm, pwmVal);
    if (dir == 1){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        }
      else if (dir == -1){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      }
      else if (dir == 0){
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        }
      else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
      }
  }

  float PID(float dt, float kp, float ki, float kd, long target, long pos, float eIntegral = 0 , float ePrev = 0 ){
    long e = pos - target;
    float dedt = (e - ePrev)/dt;
    eIntegral = eIntegral + (e*dt);
    float u = (kp*e) + (kd*dedt) + (ki*eIntegral);
    ePrev = e;
    return u;
  }
