#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 0
ICM_20948_I2C myICM;

float gyrXangle = 0.0;
float gyrYangle = 0.0;
float gyrZangle = 0.0;
float dt = 1.0/28.0;

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
    // SERIAL_PORT.print(" ");
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

void printScaledAGMT(ICM_20948_I2C *sensor){
  gyrXangle = gyrXangle - sensor->gyrX() * dt;
  gyrYangle = gyrYangle - sensor->gyrY() * dt;
  gyrZangle = gyrZangle - sensor->gyrZ() * dt;
  
  SERIAL_PORT.print("AngleX:");
  printFormattedFloat(gyrXangle,3,2);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print("AngleY:");
  printFormattedFloat(gyrYangle,3,2);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print("AngleZ:");
  printFormattedFloat(gyrZangle,3,2);
  SERIAL_PORT.println();
}
