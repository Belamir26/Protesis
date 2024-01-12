#include <Arduino.h>
#include <ICM42605_V2.h>

SPIClass* vspi = new SPIClass(VSPI);
ICM42605 IMU(*vspi,5,2);
uint8_t Ascale = AFS_2G, Gscale=GFS_250DPS, AODR=AODR_1000Hz, GODR=GODR_1000Hz;
int status;
int16_t ICM42605Data[7];


void setup() {
    Serial.begin(115200);
    while(!Serial){}
    Serial.println("Init IMU Test");
    delay(500);

    //Comunicacion con IMU
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
  
}

