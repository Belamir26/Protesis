#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_L3GD20 gyro;
void setup() {
  Serial.begin(115200);
  Wire.end();
  Wire.begin();
  gyro.begin(GYRO_RANGE_250DPS,0x69);


}

void loop() 
{
  Serial.println("Here we go: ");
  gyro.read();
  Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print("Degrees/s ");
  Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print("Degrees/s ");
  Serial.print("Z: "); Serial.println((int)gyro.data.z); Serial.print("Degrees/s ");
  delay(1000);
}
