/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 0      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

#define PRINT_ACC // Uncomment to print accelerometer data
#define PRINT_GYR // Uncomment to print gyroscope data
#define PRINT_MAG // Uncomment to print magnetometer data

ICM_20948_I2C myICM;

void setup()
{

  // Shutdown both ToF Sensors
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {};

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

void loop()
{

  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'                  
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }
  else
  {
    delay(500);
  }
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
//    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  // Absoulte values seem to help resolve graph funkiness...
  #ifdef PRINT_ACC
  SERIAL_PORT.print("AccXmg:");
  printFormattedFloat((sensor->accX()), 5, 2);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print("AccYmg:");
  printFormattedFloat((sensor->accY()), 5, 2);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print("AccZmg:");
  printFormattedFloat((sensor->accZ()), 5, 2);
  #endif

  #ifdef PRINT_GYR
  SERIAL_PORT.print("GyrXDPS:");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print("GyrYDPS:");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print("GyrZDPS:");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(", ");
  #endif

  #ifdef PRINT_MAG
  SERIAL_PORT.print("MagXuT:");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print("MagYuT:");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print("MagZuT:");
  printFormattedFloat(sensor->magZ(), 5, 2);
  #endif
  
  SERIAL_PORT.println();
}
