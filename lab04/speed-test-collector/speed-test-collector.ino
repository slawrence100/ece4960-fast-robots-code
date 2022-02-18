
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "9A48ECBA-2E92-082F-C079-9E75AAE428B1"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensorTwo(Wire, 4, INTERRUPT_PIN);

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
//////////// Global Variables ////////////

enum CommandTypes
{
  PING,
  SEND_TWO_INTS,
  SEND_THREE_FLOATS,
  ECHO,
  DANCE,
  SET_VEL,
  GET_DISTANCE
};

long currentTime = millis();
int sensorReading;

void
handle_command()
{
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     since it uses strtok internally (refer RobotCommand.h and
     https://www.cplusplus.com/reference/cstring/strtok/)
  */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }

  // Handle the command type accordingly
  switch (cmd_type) {
    /*
       Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
    */
    case PING:
      tx_estring_value.clear();
      tx_estring_value.append("PONG");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());

      break;
    /*
       Extract two integers from the command string
    */
    case SEND_TWO_INTS:
      int int_a, int_b;

      // Extract the next value from the command string as an integer
      success = robot_cmd.get_next_value(int_a);
      if (!success)
        return;

      // Extract the next value from the command string as an integer
      success = robot_cmd.get_next_value(int_b);
      if (!success)
        return;

      Serial.print("Two Integers: ");
      Serial.print(int_a);
      Serial.print(", ");
      Serial.println(int_b);

      break;
    /*
       Extract three floats from the command string
    */
    case SEND_THREE_FLOATS:
      /*
         Your code goes here.
      */
      float float_1, float_2, float_3;
      // Extract 3 floats in sequence, and fail if any one of them fails
      success = robot_cmd.get_next_value(float_1);
      if (!float_1) return;
      success = robot_cmd.get_next_value(float_2);
      if (!float_2) return;
      success = robot_cmd.get_next_value(float_3);
      if (!float_3) return;

      Serial.print("Three floats: ");
      Serial.print(float_1);
      Serial.print(", ");
      Serial.print(float_2);
      Serial.print(", ");
      Serial.println(float_3);

      break;
    /*
       Add a prefix and postfix to the string value extracted from the command string
    */
    case ECHO:

      char char_arr[MAX_MSG_SIZE];

      // Extract the next value from the command string as a character array
      success = robot_cmd.get_next_value(char_arr);
      if (!success)
        return;

      /*
         Your code goes here.
      */
      tx_estring_value.clear();
      tx_estring_value.append("Robot says: \"");
      tx_estring_value.append(char_arr);
      tx_estring_value.append("\"");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());

      break;
    /*
       DANCE
    */
    case DANCE:
      Serial.println("Look Ma, I'm Dancin'!");

      break;

    /*
       SET_VEL
    */
    case SET_VEL:

      break;

    case GET_DISTANCE:


    /*
       The default case may not capture all types of invalid commands.
       It is safer to validate the command string on the central device (in python)
       before writing to the characteristic.
    */
    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

void setup_tof(void)
{
  // Activate 1 ToF sensor
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  // Explicilty deactivate the other ToF Sensor
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  Wire.begin();
  Serial.println("VL53L1X Qwiic Test");

  // Activate the first sensor
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Sensor 1 online!");

  // NEW: Change first sensor address
  Serial.println("Changing sensor address to 0x30");
  distanceSensor.setI2CAddress(0x30);
  // ...then reactivate the other
  digitalWrite(4, HIGH);
  delay(500);
  distanceSensor.setDistanceModeLong();

}

void get_distance(SFEVL53L1X sensor, String sensorName) {
  sensor.startRanging();
  while (!sensor.checkForDataReady()) {
    delay(10);
  }
  int distance = sensor.getDistance();
  sensor.clearInterrupt();
  sensor.stopRanging();

  currentTime = millis();
  sensorReading = distance;

  tx_estring_value.clear();
  tx_estring_value.append((int)currentTime);
  tx_estring_value.append(":");
  tx_estring_value.append(sensorReading);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

void
setup()
{
  Serial.begin(115200);
  setup_tof();
  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on central devices
  tx_characteristic_float.writeValue(0.0);

  /*
     An example using the EString
  */
  // Clear the contents of the EString before using it
  tx_estring_value.clear();

  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();
}

void
write_data()
{
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

void
read_data()
{
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written()) {
    handle_command();
  }
}

void
loop()
{
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    // While central is connected
    while (central.connected()) {
      // Send data
      // write_data();

      // Read data
      read_data();

      get_distance(distanceSensor, "Sensor1");
    }

    Serial.println("Disconnected");
  }
}
