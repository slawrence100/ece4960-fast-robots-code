                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
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
#define IMU_HISTORY_LEN 300

// Motor Setup
#define MOTOR_A_FWD A15
#define MOTOR_A_BACK A16
#define MOTOR_B_BACK A2
#define MOTOR_B_FWD A3
#define MIN_POWER 40
#define MAX_POWER 127
#define COAST_POWER 10

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "9A48ECBA-2E92-082F-C079-9E75AAE428B1"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

#define BLE_UUID_TX_TOF1 "531f9dab-8ff2-4eb6-9489-c402953d27b6"
#define BLE_UUID_TX_TOF2 "1c8c3323-ca37-4f6f-a4f8-1ed5d70e0574"
#define BLE_UUID_TX_IMU "874c2554-9386-4ab0-87e2-291632899610"

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
BLEFloatCharacteristic tx_characteristic_float_tof2(BLE_UUID_TX_TOF2,
                                                    BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string_imu(BLE_UUID_TX_IMU,
                                                     BLERead | BLENotify,
                                                     MAX_MSG_SIZE);
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

int current_tof_front = -1;
int current_tof_rear = -1;
int tof_front_history[TOF_HISTORY_LEN];
unsigned long tof_front_time_history[TOF_HISTORY_LEN];
int tof_front_history_idx = 0;
float pid_motor_power[TOF_HISTORY_LEN];
int pid_motor_power_idx = 0;


// IMU array has 9 spots:
// [ Accel X, Y, Z | Gyro X Y Z | Mag X Y Z ]
float current_imu[9];
int tempMeasurement = -1;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

bool collect_data = false;
bool use_pid = false;
bool use_step_response = false;
bool use_stunt = false;
int stunt_setpoint = 300;
float stunt_proportional = 0.0;
int pid_setpoint = 300;
int step_stop = 400;
float pid_proportional = 0.0;
float pid_integral = 0.0;
float pid_derivative = 0.0;
float motor_calib_factor = 2.0;

//////////// Global Variables ////////////

enum CommandTypes {
  PING,
  SEND_TWO_INTS,
  SEND_THREE_FLOATS,
  ECHO,
  DANCE,
  SET_VEL,
  GET_FRONT_TOF,
  GET_REAR_TOF,
  GET_IMU,
  MOVE_FORWARD,
  STOP,
  START_DATA_COLLECTION,
  STOP_DATA_COLLECTION,
  GET_TOF1_DATA,
  START_PID,
  STOP_PID,
  GET_PID_DATA,
  SET_MOTOR_CALIB,
  START_STEP_RESPONSE,
  STOP_STEP_RESPONSE,
  START_STUNT,
  MOVE_BACKWARD,
  SUDDEN_FLIP
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
    /*
     * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
     */
    case PING:
      tx_estring_value.clear();
      tx_estring_value.append("PONG");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());

      break;
    /*
     * Extract two integers from the command string
     */
    case SEND_TWO_INTS:
      int int_a, int_b;

      // Extract the next value from the command string as an integer
      success = robot_cmd.get_next_value(int_a);
      if (!success) return;

      // Extract the next value from the command string as an integer
      success = robot_cmd.get_next_value(int_b);
      if (!success) return;

      Serial.print("Two Integers: ");
      Serial.print(int_a);
      Serial.print(", ");
      Serial.println(int_b);

      break;
    /*
     * Extract three floats from the command string
     */
    case SEND_THREE_FLOATS:
      break;
    /*
     * Add a prefix and postfix to the string value extracted from the command
     * string
     */
    case ECHO:
      char char_arr[MAX_MSG_SIZE];
      // Extract the next value from the command string as a character array
      success = robot_cmd.get_next_value(char_arr);
      if (!success) return;
      break;
    case DANCE:
      Serial.println("Look Ma, I'm Dancin'!");
      break;
    case SET_VEL:
      break;

    case GET_FRONT_TOF:
      tempMeasurement = get_tof_measurement(distanceSensor, true);
      Serial.print("Getting front TOF value: ");
      Serial.println(tempMeasurement);
      if (tempMeasurement != -1) {
        current_tof_front = tempMeasurement;
        tx_characteristic_float_tof1.writeValue(current_tof_front);
      }
      break;

    case GET_REAR_TOF:
      tempMeasurement = get_tof_measurement(distanceSensorTwo, true);
      Serial.print("Getting rear TOF value: ");
      Serial.println(tempMeasurement);
      if (tempMeasurement != -1) {
        current_tof_front = tempMeasurement;
      }
      tx_characteristic_float_tof2.writeValue(current_tof_rear);
      break;

    case GET_IMU:
      get_imu_measurement(&myICM, false);
      // TODO write IMU measurement to characteristic

      break;
    /*
     * The default case may not capture all types of invalid commands.
     * It is safer to validate the command string on the central device (in
     * python) before writing to the characteristic.
     */
    case MOVE_FORWARD:
      success = robot_cmd.get_next_value(base_power);
      if (!success) return;
      move_forward(clip_motor_value(base_power));
      break;

    case STOP:
      Serial.println("Stopping motors");
      stop_motors(true);
      break;
      
    case START_DATA_COLLECTION:
      collect_data = true;
      memset(tof_front_history,0,sizeof(tof_front_history));
      tof_front_history_idx = 0;
      pid_motor_power_idx = 0;
      break;

    case STOP_DATA_COLLECTION:
      collect_data = false;
      break;

    case GET_TOF1_DATA:
      success = robot_cmd.get_next_value(data_idx);
      if (!success) return;
      tx_characteristic_float_tof1.writeValue(tof_front_history[data_idx]);
      tx_characteristic_float_tof1.writeValue(float(tof_front_time_history[data_idx] / 1000));
      break;

    case START_PID:
      float proportional_in;
      success = robot_cmd.get_next_value(proportional_in);
      if (!success) return;
      pid_proportional = proportional_in;
      use_pid = true;
      break;

    case STOP_PID:
      use_pid = false;
      break;
    
    case GET_PID_DATA:
      success = robot_cmd.get_next_value(data_idx);
      if (!success) return;
      // Motor PID is bound to ToF sensor inputs
      tx_characteristic_float_motor_pid.writeValue(pid_motor_power[data_idx]);
      tx_characteristic_float_motor_pid.writeValue(float(tof_front_time_history[data_idx] / 1000));
      break;

    case SET_MOTOR_CALIB:
      success = robot_cmd.get_next_value(new_calib_factor);
      if (!success) return;
      motor_calib_factor = new_calib_factor;
      break;

    case START_STEP_RESPONSE:
      success = robot_cmd.get_next_value(step_stop);
      if (!success) return;
      use_step_response = true;
      break;
    
    case STOP_STEP_RESPONSE:
      use_step_response = false;
      break;

    case START_STUNT:
      use_stunt = true;
      success = robot_cmd.get_next_value(stunt_setpoint);
      if (!success) return;
      success = robot_cmd.get_next_value(stunt_proportional);
      if (!success) return;
      break;

    case MOVE_BACKWARD:
      success = robot_cmd.get_next_value(base_power);
      if (!success) return;
      move_backward(clip_motor_value(base_power));
      break;

    case SUDDEN_FLIP:
      success = robot_cmd.get_next_value(base_power);
      if (!success) return;
      stop_motors(true);
      move_backward(base_power);
      break;

    default:
    
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
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
  testService.addCharacteristic(tx_characteristic_float_tof2);
  testService.addCharacteristic(tx_characteristic_string_imu);
  testService.addCharacteristic(tx_characteristic_float_motor_pid);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on
  // central devices
  tx_characteristic_float.writeValue(0.0);
  tx_characteristic_float_tof1.writeValue(0.0);
  tx_characteristic_float_tof2.writeValue(0.0);

  /*
   * An example using the EString
   */
  // Clear the contents of the EString before using it
  tx_estring_value.clear();

  // Append the string literal "[->"
  tx_estring_value.append("[->");

  // Append the float value
  tx_estring_value.append(9.0);

  // Append the string literal "<-]"
  tx_estring_value.append("<-]");

  // Write the value to the characteristic
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

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
  current_imu[0] = sensor->accX();
  current_imu[1] = sensor->accY();
  current_imu[2] = sensor->accZ();

  current_imu[3] = sensor->gyrX();
  current_imu[4] = sensor->gyrY();
  current_imu[5] = sensor->gyrZ();

  current_imu[6] = sensor->magX();
  current_imu[7] = sensor->magY();
  current_imu[8] = sensor->magZ();
  
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
    
    // While central is connected
    while (central.connected()) {
      // Send data
      // write_data();
        
      // Read data
      read_data();

      if (collect_data) {
        tempMeasurement = get_tof_measurement(distanceSensorTwo, true);
        if (tempMeasurement != -1) {
          current_tof_front = tempMeasurement;
          tof_front_history[tof_front_history_idx] = tempMeasurement;
          tof_front_time_history[tof_front_history_idx] = micros();
          tof_front_history_idx++;
        }

        if (use_pid) {
          // TOF faces backwards, so move backwards too
          motor_power = pid_proportional * (current_tof_front - pid_setpoint);
          pid_motor_power[pid_motor_power_idx] = motor_power;
          pid_motor_power_idx++;
          if (motor_power >= MIN_POWER) {
            move_forward(clip_motor_value(motor_power));
          } else if (motor_power >= COAST_POWER){
            stop_motors(false); // coast if we need some movement but not a lot
          } else {
            stop_motors(true); // close enough to equal, so make a hard stop
          }
        }

        if (use_step_response) {
          // Again, use backwards-facing ToF
          motor_power = clip_motor_value(255);
          if (current_tof_front <= step_stop) {
            motor_power = 0;
            stop_motors(true);
          } else {
            move_forward(motor_power);
          }
          pid_motor_power[pid_motor_power_idx] = motor_power;
          pid_motor_power_idx++;
        }

        if (use_stunt) {
        // TOF faces backwards, so move backwards too
        motor_power = stunt_proportional * (current_tof_front - stunt_setpoint);
        pid_motor_power[pid_motor_power_idx] = motor_power;
        pid_motor_power_idx++;
        if (motor_power >= MIN_POWER) {
          move_forward(clip_motor_value(motor_power));
        } else if (motor_power >= COAST_POWER){
          stop_motors(false); // coast if we need some movement but not a lot
        } else {
          stop_motors(true); // close enough to equal, so make a hard stop
          move_backward(clip_motor_value(255));
          delay(1000);
          stop_motors(true);
          use_stunt = false;
        }
      }
      }

      
    }

    Serial.println("Disconnected");
  }
}
