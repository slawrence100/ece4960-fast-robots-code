
#define MOTOR_A_FWD A15
#define MOTOR_A_BACK A16
#define MOTOR_B_FWD A2
#define MOTOR_B_BACK A3

#define MAX_ANALOG 255

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_A_FWD, OUTPUT);
  pinMode(MOTOR_A_BACK, OUTPUT);
  pinMode(MOTOR_B_FWD, OUTPUT);
  pinMode(MOTOR_B_BACK, OUTPUT);
  
}

void loop() {
  
  digitalWrite(MOTOR_A_FWD, HIGH);
  digitalWrite(MOTOR_A_BACK, LOW);
  delay(1000);
  digitalWrite(MOTOR_A_FWD, LOW);
  digitalWrite(MOTOR_A_BACK, LOW);
  delay(5000);

  digitalWrite(MOTOR_B_FWD, HIGH);
  digitalWrite(MOTOR_B_BACK, LOW);
  delay(1000);
  digitalWrite(MOTOR_B_FWD, LOW);
  digitalWrite(MOTOR_B_BACK, LOW);
  delay(5000);
  
}
