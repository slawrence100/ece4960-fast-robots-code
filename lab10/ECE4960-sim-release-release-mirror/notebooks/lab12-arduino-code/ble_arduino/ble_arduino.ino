// Lab 12: Real Localization                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
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
#define IMU_HISTORY_LEN 2400
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

bool collect_data = false;
bool use_pid = false;
bool use_step_response = false;
bool use_stunt = false;
int stunt_setpoint = 300;
float stunt_proportional = 0.0;
float pid_setpoint = 300.0;
int pid_min_power = 100;
int step_stop = 400;
float pid_proportional = 0.0;
float pid_integral = 0.0;
float pid_derivative = 0.0;
float motor_calib_factor = 2.0;

//////////// Global Variables ////////////

enum CommandTypes {
  SET_MOTOR_CALIB,
  SET_PID,
  OBSERVE,
  GET_DATA
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
      success = robot_cmd.get_next_value(new_calib_factor);
      if (!success) return;
      motor_calib_factor = new_calib_factor;
      break;

    case SET_PID:
      float proportional_in;
      success = robot_cmd.get_next_value(proportional_in);
      if (!success) return;
      success = robot_cmd.get_next_value(pid_min_power);
      if (!success) return;
      success = robot_cmd.get_next_value(imu_dt);
      if (!success) return;
      imu_dt = 1.0 / imu_dt;
      pid_proportional = proportional_in;
      use_pid = true;
      first_imu = true;
      break;
    
    case OBSERVE:
      for (int i = 0; i < 18; i++) {
        turn_degrees(-20);
        for (int j = 0; j < 3; j++) {
          tempMeasurement += get_tof_measurement(distanceSensorTwo, true);
        }
        sensor_meas[i] = tempMeasurement / 3.0;
        angle_meas[i] = pitch;
        tempMeasurement = 0;
      }
      break;

    case GET_DATA:
      for (int i = 0; i < 18; i++) {
        Serial.println("Writing data point ");
        Serial.println(i);
        Serial.println("of ");
        Serial.println(sensor_meas[i]);
        tx_characteristic_float_tof1.writeValue(sensor_meas[i]);
        tx_characteristic_float_imu_pitch.writeValue(angle_meas[i]);
      }
      break;
      
    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

void turn_degrees(int deg) {
  pitch = 0;
  first_imu = true;
  while (true) {
    get_imu_measurement(&myICM, true);
    Serial.println("Angle: ");
    Serial.print(pitch);
    int motor_power = pid_proportional * (pitch - deg);
    if (motor_power >= 200 || motor_power <= -200) {
      Serial.print("Spinning at max power: ");
      Serial.println(motor_power);
      motor_power = 200 * ((motor_power < 0) ? -1 : 1);
      spin(motor_power, -1*motor_power * motor_calib_factor);
    } else if (abs(pitch - deg) < 0.5) {
      Serial.print("Stopping to measure");
      // close enough to equal, so make a hard stop
      stop_motors(true);
      return;
    } else {
      motor_power = ((motor_power < 0) ? -1 : 1) * max(abs(pid_min_power), abs(motor_power));
      spin(motor_power, -1 * motor_power * motor_calib_factor);
      Serial.print("Spinning: ");
      Serial.println(motor_power);
    }
  }
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
 * @param blocking Wait for a new reading if true
 * @return -1 if there's no new value, or the distance in millimeters
 */
int get_tof_measurement(SFEVL53L1X sensor, bool blocking) {
  sensor.startRanging();
  int distance;

  if (blocking) {
    while (!sensor.checkForDataReady()) {};
    distance = sensor.getDistance();
    sensor.clearInterrupt();
    sensor.stopRanging();
    return distance;
  }

  if (sensor.checkForDataReady()) {
    distance = sensor.getDistance();
    sensor.clearInterrupt();
    sensor.stopRanging();
    return distance;
  }
  return -1;
}

void move_backward(int base_power) {
  analogWrite(MOTOR_A_BACK, base_power);
  analogWrite(MOTOR_B_BACK, base_power * motor_calib_factor);

  analogWrite(MOTOR_A_FWD, 0);
  analogWrite(MOTOR_B_FWD, 0);
}

void move_forward(int base_power) {
  analogWrite(MOTOR_A_FWD, base_power);
  analogWrite(MOTOR_B_FWD, base_power * motor_calib_factor);

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
 * @return The clipped value
 */
int clip_motor_value(float val_in) {
  if (val_in >= (255 / motor_calib_factor) ){
    return int(255 / motor_calib_factor);
  } else if (val_in <= MIN_POWER) {
    return 0;
  }
  return round(val_in);
}

void loop() {
  // Listen for connections
  BLEDevice central = BLE.central();
  
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
