#include <Arduino.h>
#include <ICM42605.h>

SPIClass* vspi = new SPIClass(VSPI);
ICM42605 IMU(*vspi,5,2);
int status;
uint8_t Ascale = AFS_2G, Gscale=GFS_250DPS, AODR=AODR_1000Hz, GODR=GODR_1000Hz;
int16_t ICM42605Data[3];


void setup() {
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("Init IMU Test");
  delay(500);

  //Comunicacicon con imu
  status =IMU.begin(Ascale,Gscale,AODR,GODR);
  if(status < 0){
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  
  
  /*
  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
  IMU.readSensor(ICM42605Data);
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
  delay(50);
  
  */

  
  

 /*
 IMU.readApex(ICM42605Data);
 Serial.print("Step:  ");
 Serial.print(IMU.getStepCount(),6);
 Serial.print("Cadence");
 Serial.print(IMU.getCadence(),6);
 Serial.println();
 delay(50); 
 */
 

  
}

