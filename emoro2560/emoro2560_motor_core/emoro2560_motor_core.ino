//motor A definitions
int bin1 = PWM_2;
int bin2 = PWM_1;
int pwmB = PWM_0;
//motor B definitions
int ain1 = PWM_3;
int ain2 = PWM_4;
int pwmA = PWM_5;


void initMot() {
  //motor A pinMode
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  //motor B pinMode
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(pwmB, OUTPUT);
  // Set direction to no-direction
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, HIGH);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, HIGH);
}

void LMotorControl(int in){
  if (in > 0) {              // forward
    digitalWrite(ain2, HIGH);
    digitalWrite(ain1, LOW);
  }
  else if(in < 0){                     // backward
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  }
  else if(in==0){
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, HIGH);
  }
  byte vel = abs(in);
  analogWrite(pwmA, vel); 
}

void RMotorControl(int in){
  if (in > 0) {              // forward
    digitalWrite(bin2, HIGH);
    digitalWrite(bin1, LOW);
  }
  else if(in < 0){                     // backward
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  }
  else if(in==0){
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, HIGH);
  }
  byte vel = abs(in);
  analogWrite(pwmB, vel); 
}

void setup() {
  // put your setup code here, to run once:
  initMot();
  LMotorControl(-255);
  delay(1000);
  LMotorControl(255);
  delay(1000);
  LMotorControl(0);
  delay(1000);
  
  RMotorControl(-255);
  delay(1000);
  RMotorControl(255);
  delay(1000);
  RMotorControl(0);
  delay(1000);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
