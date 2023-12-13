#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Wire.h>

 //Toca definir el addres y busca en el codigo cual es cual
  //Este sensor hace dos cosas accelerometro y magnetismo
//Objetos
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(0x1D);

//Para mostar lo del sensor en el serial 
void disAtSerial(void){
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Accelerometer Test"); Serial.println("");
  Wire.end();
  Wire.begin();
  accel.begin();
  disAtSerial();
}

void loop() {
  Serial.print("Here we go:  ");
  sensors_event_t event;

  accel.getEvent(&event); 


  // Este es la medida.. en XYZ
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s");



  Serial.print("X: RAW "); Serial.print(accel.raw.x); Serial.print("  ");
  Serial.print("Y: RAW"); Serial.print(accel.raw.y); Serial.print("  ");
  Serial.print("Z: RAW"); Serial.print(accel.raw.z); Serial.print("  ");Serial.println("  ");

  //Pilas que tambien puedo acquirir el valor crudo o RAW.. 
  delay(1000);

}

