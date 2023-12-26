#include <Arduino.h>
#include "ICM42605.h"

// an ICM20689 object with the ICM20689 sensor on SPI bus 0 and chip select pin 10
SPIClass* vspi = new SPIClass(VSPI);
ICM42605 IMU(*vspi,5);
int status;
uint8_t Ascale = AFS_2G, Gscale = GFS_250DPS, AODR = AODR_1000Hz, GODR = GODR_1000Hz;
int16_t ICM42605Data[7];        // Stores the 16-bit signed sensor output

void setup() {
  // serial to display data
  Serial.begin(115200);
  delay(1000);
  while(!Serial) {}
  Serial.println("Init IMU Test");
  delay(1000);

  // start communication with IMU
  status = IMU.begin(Ascale, Gscale, AODR, GODR);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
}

void loop() {
  // read the sensor
  IMU.readSensor(ICM42605Data);
  // display the data
  Serial.print(IMU.getAccelX(),6);
  Serial.print(",");
  Serial.print(IMU.getAccelY(),6);
  Serial.print(",");
  Serial.print(IMU.getAccelZ(),6);
  Serial.print(",");
  Serial.print(IMU.getGyroX(),6);
  Serial.print(",");
  Serial.print(IMU.getGyroY(),6);
  Serial.print(",");
  Serial.print(IMU.getGyroZ(),6);
  Serial.print(",");
  Serial.println(IMU.getTemperature_C(),6);
  delay(100);
}
