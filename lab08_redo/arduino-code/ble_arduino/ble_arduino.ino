// Lab 8 Redo: stunts                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
#include <ArduinoBLE.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"

// ToF Imports
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <Wire.h>
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>

#include "SparkFun_VL53L1X.h"

// IMU imports
#include "ICM_20948.h"

// TOF Setup
#define INTERRUPT_PIN 3
#define SHUTDOWN_ONE A5
#define SHUTDOWN_TWO 4
#define TOF_HISTORY_LEN 300

// IMU Setup
#define AD0_VAL 0
#define WIRE_PORT Wire
#define SERIAL_PORT Serial
#define PITCH_CONSTANT 0.05
#define ROLL_CONSTANT 0.05

// Motor Setup
#define MOTOR_A_FWD A15
#define MOTOR_A_BACK A16
#define MOTOR_B_BACK A2
#define MOTOR_B_FWD A3
#define MIN_POWER 40
#define MAX_POWER 127
#define COAST_POWER 10

// Other constants
#define PID_HISTORY_LEN 300

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "6a9e1931-4e08-4301-820e-0fa0f540d922"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

#define BLE_UUID_TX_TOF1 "531f9dab-8ff2-4eb6-9489-c402953d27b6"
#define BLE_UUID_TX_IMU_PITCH "5eeb1742-ebe3-48a9-8fa7-de93585bd484"

#define BLE_UUID_TX_MOTOR_PID "459c548e-87b6-4dd0-834d-e990278b9393"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);
BLEDevice central;

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite,
                                                  MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT,
                                               BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING,
                                                  BLERead | BLENotify,
                                                  MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float_tof1(BLE_UUID_TX_TOF1,
                                                    BLERead | BLENotify);
BLEFloatCharacteristic tx_characteristic_float_imu_pitch(BLE_UUID_TX_IMU_PITCH,
                                                    BLERead | BLENotify);
BLEFloatCharacteristic tx_characteristic_float_motor_pid(BLE_UUID_TX_MOTOR_PID,
                                                    BLERead | BLENotify);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;
float tx_float_tof1 = 0.0;
float tx_float_tof2 = 0.0;
EString imu_value;

// Sensors
SFEVL53L1X distanceSensor(Wire, SHUTDOWN_ONE, INTERRUPT_PIN);
SFEVL53L1X distanceSensorTwo(Wire, SHUTDOWN_TWO, INTERRUPT_PIN);
ICM_20948_I2C myICM;

// Be able to store 18 measurements 20 degrees apart
float sensor_meas[18];
float angle_meas[18];

float pid_history[PID_HISTORY_LEN];
float pid_time_history[PID_HISTORY_LEN];
float tof_history[TOF_HISTORY_LEN];
float tof_time_history[TOF_HISTORY_LEN];
int pid_idx = 0; int tof_idx = 0;


int tempMeasurement = -1;

// IMU array has 9 spots:
// [ Accel X, Y, Z | Gyro X Y Z | Mag X Y Z ]
int imu_history_idx = 0;
float pitch;
float roll;
bool first_imu = true;
float imu_dt = 1.0/60;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

// Rotational variables (rot)
int pid_min_power_rot = 100;
float pid_proportional_rot = 0.0;
float motor_calib_factor_rot = 2.0;

// Translational variables (fwd)
int pid_min_power_fwd = 100;
float pid_proportional_fwd = 0.0;
float motor_calib_factor_fwd = 2.0;

//////////// Global Variables ////////////

enum CommandTypes {
  SET_MOTOR_CALIB,
  SET_PID,
  MOVE_DISTANCE,
  TURN_DEGREES,
  STOP,
  DO_STUNT,
  GET_DATA,
  SPIN,
  MOVE_DURATION
};

void handle_command() {
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;
  int data_idx;
  float new_calib_factor;
  int base_power;
  int data_type;
  int spin_left, spin_right;
  float temp_avg;
  int temp_move;
  int temp_move2;
  int temp_time;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
   * since it uses strtok internally (refer RobotCommand.h and
   * https://www.cplusplus.com/reference/cstring/strtok/)
   */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }
  
  // Handle the command type accordingly
  switch (cmd_type) {

    case SET_MOTOR_CALIB:
      success = robot_cmd.get_next_value(motor_calib_factor_rot);
      if (!success) return;
      success = robot_cmd.get_next_value(motor_calib_factor_fwd);
      if (!success) return;
      break;

    case SET_PID:
      success = robot_cmd.get_next_value(pid_proportional_rot);
      if (!success) return;
      success = robot_cmd.get_next_value(pid_min_power_rot);
      if (!success) return;
      success = robot_cmd.get_next_value(pid_proportional_fwd);
      if (!success) return;
      success = robot_cmd.get_next_value(pid_min_power_fwd);
      if (!success) return;
      success = robot_cmd.get_next_value(imu_dt);
      if (!success) return;
      imu_dt = 1.0 / imu_dt;
      first_imu = true;
      pid_idx = 0; tof_idx = 0;
      break;
      
    case MOVE_DISTANCE:
      success = robot_cmd.get_next_value(temp_move);
      if (!success) return;
      move_distance(temp_move);
      return;
    
    case TURN_DEGREES:
      success = robot_cmd.get_next_value(temp_move);
      if (!success) return;
      turn_degrees(temp_move);
      return;

    case STOP:
      stop_motors(true);
      break;

    case DO_STUNT:
      success = robot_cmd.get_next_value(temp_move);
      if (!success) return;
      drive_until(temp_move);
      move_backward(255 / motor_calib_factor_fwd);
      delay(1500);
      stop_motors(true);
      break;

    case GET_DATA:
      Serial.println("Sending data...");
      for (int i = 0; i < TOF_HISTORY_LEN; i++) {
        tx_characteristic_float_tof1.writeValue(tof_history[i]);
      }
      for (int i = 0; i < PID_HISTORY_LEN; i++) {
        tx_characteristic_float_motor_pid.writeValue(pid_history[i]);
      }
      for (int i = 0; i < PID_HISTORY_LEN; i++) {
        tx_characteristic_float_imu_pitch.writeValue(pid_time_history[i]);
      }
      Serial.println("Done sending data");
      break;

    case SPIN:
      success = robot_cmd.get_next_value(spin_left);
      if (!success) return;
      success = robot_cmd.get_next_value(spin_right);
      if (!success) return;
      success = robot_cmd.get_next_value(temp_move);
      if (!success) return;
      spin(spin_left, spin_right);
      delay(temp_move);
      stop_motors(true);
      break;

    case MOVE_DURATION:
      success = robot_cmd.get_next_value(temp_move);
      if (!success) return;
      success = robot_cmd.get_next_value(temp_time);
      if (!success) return;
      if (temp_move > 0){
        move_forward(temp_move);
      } else {
        move_backward(temp_move);
      }
      delay(temp_time);
      stop_motors(true);
      break;

    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}
/**
 * Turn the robot on its axis
 * 
 * @param deg The number of degrees to turn by. A negative angle indicates a
 * counter-clockwise rotation.
 */
void turn_degrees(int deg) {
  pitch = 0;
  first_imu = true;
  while (true) {
    get_imu_measurement(&myICM, true);
    int motor_power = pid_proportional_rot * (pitch - deg);
    if (motor_power >= 200 || motor_power <= -200) {
      motor_power = 200 * ((motor_power < 0) ? -1 : 1);
      spin(motor_power, -1*motor_power * motor_calib_factor_rot);
    } else if (abs(pitch - deg) < 0.5) {
      // close enough to equal, so make a hard stop
      stop_motors(true);
      return;
    } else {
      motor_power = ((motor_power < 0) ? -1 : 1) * max(abs(pid_min_power_rot), abs(motor_power));
      spin(motor_power, -1 * motor_power * motor_calib_factor_rot);
    }
  }
}

/**
 * Move forwards.
 * @param dist The distance to move forwards, in mm
 */
void move_distance(int dist) {
  int tof_dist = get_tof_measurement(distanceSensorTwo, 5);
  // setpoint is when the sensor reads the same as it would have in the target's place  
  int target = tof_dist - dist;
  int motor_power;
  int coast_power = 20;

  while (true) {
    tof_dist = get_tof_measurement(distanceSensorTwo, 1);
    motor_power = pid_proportional_fwd * (tof_dist - target);
    if (motor_power < -1*pid_min_power_fwd) {
      move_backward(motor_power);
    } else if (motor_power > -1*pid_min_power_fwd && motor_power < -1*coast_power) {
      stop_motors(false);
    } else if (motor_power > -1*coast_power && motor_power < coast_power) {
      stop_motors(true);
      return;
    } else if (motor_power > coast_power && motor_power < pid_min_power_fwd) {
      stop_motors(false);
    } else if (motor_power > pid_min_power_fwd) {
      move_forward(motor_power);
    }
  }
}

void drive_until(int tof_target) {
  int tof_dist = get_tof_measurement(distanceSensorTwo, 5);
  int motor_power;
  int coast_power = 20;
  while (true) {
    tof_dist = get_tof_measurement(distanceSensorTwo, 1);
    motor_power = pid_proportional_fwd * (tof_dist - tof_target);
    record_data(tof_dist, motor_power);
    if (motor_power < -1*pid_min_power_fwd) {
      move_backward(motor_power);
    } else if (motor_power > -1*pid_min_power_fwd && motor_power < -1*coast_power) {
      stop_motors(false);
    } else if (motor_power > -1*coast_power && motor_power < coast_power) {
      stop_motors(true);
      return;
    } else if (motor_power > coast_power && motor_power < pid_min_power_fwd) {
      stop_motors(false);
    } else if (motor_power > pid_min_power_fwd) {
      move_forward(motor_power);
    }
  }
}

void record_data(float tof, float pid){
  tof_history[tof_idx] = tof;
  pid_history[pid_idx] = pid;
  pid_time_history[pid_idx] = (float)millis();
  tof_idx++;
  pid_idx++;
}

void setup_sensors() {
  /////////// ToF ///////////////

  // Activate 1 ToF sensor
  pinMode(SHUTDOWN_ONE, OUTPUT);
  digitalWrite(SHUTDOWN_ONE, HIGH);

  // Explicilty deactivate the other ToF Sensor
  pinMode(SHUTDOWN_TWO, OUTPUT);
  digitalWrite(SHUTDOWN_TWO, LOW);
                            
  Wire.begin();
  Serial.println("VL53L1X Qwiic Test");

  // Activate the first sensor
  if (distanceSensor.begin() != 0)  // Begin returns 0 on a good init
  {
    Serial.println(
        "Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Sensor 1 online!");

  // NEW: Change first sensor address
  Serial.println("Changing sensor address to 0x30");
  distanceSensor.setI2CAddress(0x30);

  // ...then reactivate the other
  digitalWrite(SHUTDOWN_TWO, HIGH);
  delay(250);

  if (distanceSensorTwo.begin() != 0)  // Begin returns 0 on a good init
  {
    Serial.println(
        "Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 2 online!");

  // distanceSensor.setProxIntegrationTime(2);
  
  delay(250);
  /////////// End ToF ///////////////

  /////////// IMU ///////////////////

  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Waiting for IMU to initialize");
      delay(500);
    } else {
      initialized = true;
    }
  }
  
  Serial.println("IMU Ready!");

  for (int i = 0; i < TOF_HISTORY_LEN; i++){
    tof_history[i] = i;
  }
  for (int i = 0; i < PID_HISTORY_LEN; i++) {
    pid_history[i] = i;
  }
  for (int i = 0; i < PID_HISTORY_LEN; i++){
    pid_time_history[i] = i;
  }

  Serial.println("All sensors are ready!");
}

void setup_motors() {
  pinMode(MOTOR_A_FWD, OUTPUT);
  pinMode(MOTOR_A_BACK, OUTPUT);
  pinMode(MOTOR_B_FWD, OUTPUT);
  pinMode(MOTOR_B_BACK, OUTPUT);
}

void setup() {
  Serial.begin(115200);

  setup_sensors();

  setup_motors();

  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);
  testService.addCharacteristic(tx_characteristic_float_tof1);
  testService.addCharacteristic(tx_characteristic_float_imu_pitch);
  testService.addCharacteristic(tx_characteristic_float_motor_pid);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on
  // central devices
  tx_characteristic_float.writeValue(0.0);
  tx_characteristic_float_tof1.writeValue(0.0);
  tx_characteristic_float_imu_pitch.writeValue(0.0);
  
  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();
}

/**
 * @brief Get a measurement from the IMU
 * 
 * @param blocking Wait for a new measurement if true
 */
void get_imu_measurement(ICM_20948_I2C* sensor, bool blocking) {
  if (blocking) {
    while (!sensor->dataReady()) {}
  }
  myICM.getAGMT();
  float accPitch = atan2(sensor->accX(), sensor->accZ())* 360 / (2 * M_PI);
  float accRoll = atan2(sensor->accY(), sensor->accZ()) * 360 / (2 * M_PI);
  float gyrPitch = sensor->gyrX();
  float gyrRoll = sensor->gyrY();
  
  if (first_imu) {
    pitch = 0 - gyrPitch * imu_dt;
    roll = 0 - gyrRoll * imu_dt;
    first_imu = false;
  } else {
    pitch = (pitch + gyrPitch * imu_dt);
    roll = (roll + gyrRoll * imu_dt);
  }
}

/**
 * @brief Try to get a Time of Flight sensor measurment
 * 
 * @param sensor The sensor to read from
 * @param nTries The number of times the sensor should read before returning a value
 * @return -1 if there's no new value, or the distance in millimeters
 */
int get_tof_measurement(SFEVL53L1X sensor, int nTries) {
  sensor.startRanging();
  int distance = 0;
  for (int i = 0; i < nTries; i++){
    while (!sensor.checkForDataReady()) {};
    distance += sensor.getDistance();
    sensor.clearInterrupt();
    sensor.stopRanging();
    sensor.startRanging();
  }

  return distance / nTries;

}

void move_backward(int base_power) {
  analogWrite(MOTOR_A_BACK, base_power);
  analogWrite(MOTOR_B_BACK, base_power * motor_calib_factor_fwd);

  analogWrite(MOTOR_A_FWD, 0);
  analogWrite(MOTOR_B_FWD, 0);
}

void move_forward(int base_power) {
  analogWrite(MOTOR_A_FWD, base_power);
  analogWrite(MOTOR_B_FWD, base_power * motor_calib_factor_fwd);

  analogWrite(MOTOR_A_BACK, 0);
  analogWrite(MOTOR_B_BACK, 0);
}

void spin(int spin_left, int spin_right) {
  if (spin_left < 0){
    analogWrite(MOTOR_A_FWD, 0);
    analogWrite(MOTOR_A_BACK, -1 * spin_left);
  } else {
    analogWrite(MOTOR_A_BACK, 0);
    analogWrite(MOTOR_A_FWD, spin_left);
  }
  if (spin_right < 0) {
    analogWrite(MOTOR_B_FWD, 0);
    analogWrite(MOTOR_B_BACK, -1 * spin_right);
  } else {
    analogWrite(MOTOR_B_BACK, 0);
    analogWrite(MOTOR_B_FWD, spin_right);
  }
}

void stop_motors(bool hard_brake){
  int power = 0;
  if (hard_brake) {
    power = 255;
  }
  analogWrite(MOTOR_A_BACK, power);
  analogWrite(MOTOR_A_FWD, power);
  analogWrite(MOTOR_B_BACK, power);
  analogWrite(MOTOR_B_FWD, power);
}

void write_data() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);

    if (tx_float_value > 10000) {
      tx_float_value = 0;
    }

    previousMillis = currentMillis;
  }
}

void read_data() {
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written()) {
    handle_command();
  }
}

/**
 * @brief Clip the motor value so it's always valid
 * 
 * @param val_in The value to clip
 * @param calib The calibration factor to use (_fwd or _rot)
 * @return The clipped value
 */
int clip_motor_value(float val_in, float calib) {
  if (val_in >= (255 / calib) ){
    return int(255 / calib);
  } else if (val_in <= MIN_POWER) {
    return 0;
  }
  return round(val_in);
}

void loop() {
  // Listen for connections
  central = BLE.central();
  
  // If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    int motor_power = 0;
    int past_power = -1;
    int tof_meas;

    // While central is connected
    while (central.connected()) {
      // Send data
      // write_data();
        
      // Read data
      read_data();
      
    }

    Serial.println("Disconnected");
  }
}
