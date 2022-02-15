
#include "Wire.h"

int xshut_pin = 8;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting up");
  pinMode(xshut_pin, OUTPUT);
  digitalWrite(xshut_pin, LOW);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
