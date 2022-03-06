#define MOTOR_A_FWD A15
#define MOTOR_A_BACK A16
#define MOTOR_B_BACK A2
#define MOTOR_B_FWD A3

#define MAX_ANALOG 255

#define CALIBRATION_FACTOR 2
#define TURN_FACTOR 2

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_A_FWD, OUTPUT);
  pinMode(MOTOR_A_BACK, OUTPUT);
  pinMode(MOTOR_B_FWD, OUTPUT);
  pinMode(MOTOR_B_BACK, OUTPUT);
  
}

void loop() {
  delay(5000);
  test_turns(50);
  // go_forward(60);
}

void test_turns(int base_power) {

  // Forward
  digitalWrite(MOTOR_A_BACK, LOW);
  analogWrite(MOTOR_A_FWD, base_power);
  digitalWrite(MOTOR_B_BACK, LOW);
  analogWrite(MOTOR_B_FWD, base_power * CALIBRATION_FACTOR);
  delay(500);
  stop();

  // A-turn
  digitalWrite(MOTOR_A_BACK, LOW);
  analogWrite(MOTOR_A_FWD, base_power * TURN_FACTOR);
  digitalWrite(MOTOR_B_BACK, LOW);
  digitalWrite(MOTOR_B_FWD, LOW);
  delay(500 * TURN_FACTOR);
  stop();

  // B-Turn
  digitalWrite(MOTOR_A_BACK, LOW);
  digitalWrite(MOTOR_A_FWD, LOW);
  digitalWrite(MOTOR_B_BACK, LOW);
  analogWrite(MOTOR_B_FWD, base_power * TURN_FACTOR * CALIBRATION_FACTOR);
  delay(500 * TURN_FACTOR);
  stop();

  // Forward
  digitalWrite(MOTOR_A_BACK, LOW);
  analogWrite(MOTOR_A_FWD, base_power);
  digitalWrite(MOTOR_B_BACK, LOW);
  analogWrite(MOTOR_B_FWD, base_power * CALIBRATION_FACTOR);
  delay(500);
  stop();
  
  delay(2000);
}

void go_forward(int base_power){
  digitalWrite(MOTOR_A_BACK, LOW);
  analogWrite(MOTOR_A_FWD, base_power);
  digitalWrite(MOTOR_B_BACK, LOW);
  analogWrite(MOTOR_B_FWD, base_power * CALIBRATION_FACTOR);
  delay(3500);
  analogWrite(MOTOR_A_FWD, 0);
  analogWrite(MOTOR_B_FWD, 0);
}

void test_individual() {
  // A Forward
  digitalWrite(MOTOR_A_BACK, LOW);
  analogWrite(MOTOR_A_FWD, 40);
  delay(1000);
  analogWrite(MOTOR_A_FWD, 0);
  delay(3000);

  // A, Backward
  digitalWrite(MOTOR_A_FWD, LOW);
  analogWrite(MOTOR_A_BACK, 40);
  delay(1000);
  analogWrite(MOTOR_A_BACK, 0);
  delay(3000);

  // B, Forward
  digitalWrite(MOTOR_B_BACK, LOW);
  analogWrite(MOTOR_B_FWD, 40 * CALIBRATION_FACTOR);
  delay(1000);
  analogWrite(MOTOR_B_FWD, 0);
  delay(3000);

  // B, Backward
  digitalWrite(MOTOR_B_FWD, LOW);
  analogWrite(MOTOR_B_BACK, 40 * CALIBRATION_FACTOR);
  delay(1000);
  analogWrite(MOTOR_B_BACK, 0);
  delay(3000);
}

void stop() {
  analogWrite(MOTOR_A_BACK, 0);
  analogWrite(MOTOR_A_FWD, 0);
  analogWrite(MOTOR_B_BACK, 0);
  analogWrite(MOTOR_B_FWD, 0);
  delay(500);
}
