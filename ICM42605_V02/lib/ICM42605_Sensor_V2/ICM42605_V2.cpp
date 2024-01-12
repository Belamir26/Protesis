#include "ICM42605_V2.h"


//Public
/*ICM42605 Object to SPI and Chip select pin*/
ICM42605::ICM42605(SPIClass &bus, uint8_t csPin, uint8_t APEX)
{
    _spi= &bus;
    _csPin = csPin;
    _useSPI = true;
    if(APEX==0){_useRegular=true;}
    if(APEX==1){_usePedom=true;} 
    if(APEX==2){_useTap=true;} 

}
/*Initalice the IMU's Object*/
int ICM42605::begin(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
    aRes= getAres(Ascale);
    gRes= getGres(Gscale);

    pinMode(_csPin,OUTPUT);     // setting CS pin to output
    digitalWrite(_csPin,HIGH);
    _spi->begin();              // begin SPI communication
    reset();

    // check the WHO AM I byte, expected value is 0x42 (decimal 66)
    if(whoAreWe() !=0x42){
        return -1;
    }
    

    //Regular Accel, Gyro & Temp
    if(_useRegular){
      // enable gyro and accel in low noise mode
      if(writeAll(ICM42605_PWR_MGMT0,0x0F) < 0) { 
          return -2;
      }
      // gyro full scale and data rate
      if(writeAll(ICM42605_GYRO_CONFIG0, GODR | Gscale << 5 ) < 0) { 
          return -3;
      }
      // set accel full scale and data rate
      if(writeAll(ICM42605_ACCEL_CONFIG0, AODR | Ascale << 5) < 0) {
          return -4;
      }
      // set temperature sensor low pass filter to 5Hz, use first order gyro filter
      if(writeAll(ICM42605_GYRO_CONFIG1,0xD0) < 0) {
          return -5;
      }
      //Calculate Bias
      offsetBias(accelBias, gyroBias);
    }

    //Set Tap's APEX feature
    if(_useTap){
      Serial.println("TAP FEATURE ACTIVE");
      //Initize Sensor..
      if(writeBits(ICM42605_ACCEL_CONFIG0,AODR_1000Hz,0b0000111)<0){  //ACCEL_ODR=1kHZ, 0x50
        return 200;
      }
      if(writeBits(ICM42605_PWR_MGMT0,1,0b00000011)<0){               //Accel_mode=1, 0x4E
        return 201;
      }
      if(writeBits(ICM42605_ACCEL_CONFIG1,2<<3,0b00011000)<0){        //accel_ui_filt_ord=2 0x53 bank0
        return 202;
      }
      if(writeBits(ICM42605_GYRO_ACCEL_CONFIG0,0<<4,0b11110000)<0){   // accel_ui_gilt_bw=0 0x52 bank0
        return 203;
      }
      delay(1);                                                       //wait 1m
                                                             
      //Initialize APEX Hardware...
      /* tengo que apagar el accel and gyro... COMO???*/

      //turn off
      Serial.println("TURN OFF GYRO AND ACCEL");
      Serial.println();
      Serial.println();
      writeBits(ICM42605_PWR_MGMT0,0,0b00000011);
      writeBits(ICM42605_PWR_MGMT0,0x47,0b00001100);

      /*ESTOS NO COGE*/ 
      writeBits(ICM42605_APEX_CONFIG8, 2<<5,0b01100000);        //Tap_TMAX to 2 ,0X47
      writeBits(ICM42605_APEX_CONFIG8,3,0b00000111);            //TAP_TMIN to 3 ,0x47
      writeBits(ICM42605_APEX_CONFIG8,3,0b00011000);            //TAP_TAVG to 3, 0x47
      writeBits(ICM42605_APEX_CONFIG7,17<<2,0b11111100);        //TAP_MIN_JERK_THR to 17, 0x46
      writeBits(ICM42605_APEX_CONFIG7,2,0b00000011);            //TAP_MAX_PEAK_TOL to 2, 0x46
      /* ESTOS NO COGE*/
      Serial.println("TURN ON GYRO AND ACCEL");   
      Serial.println();
      Serial.println();
      writeBits(ICM42605_PWR_MGMT0,3,0b00000011);    // low noise
      writeBits(ICM42605_PWR_MGMT0,0<<2,0b00001100);  // Gyro no se requiere
      
      delay(1);                                                 //Wait 1mili
      if(writeBits(ICM42605_INT_SOURCE6,1,0b00000001)<0){       //Enable TAP source ->  INT_SOURCE6(bit-0) to 1  0x4D
        return 209;
      }                                                     
      delay(50);                                                //Wait 501mili
      if(writeBits(ICM42605_APEX_CONFIG0,1<<6,0b01000000)<0){   //Turn on TAP feature -> TAP_Enable to 1  0x56
        return 210;
      }                                              
    }
    return 1;
}
/*Reset Device*/
void ICM42605::reset()
{
    writeAll(ICM42605_DEVICE_CONFIG,0x01);     // reset the ICM42605
    delay(1);                                       // wait for ICM42605 to come back up
}
/*Set Bias*/
void ICM42605::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(4000);

  for (int ii = 0; ii < 128; ii++)
  {
    readSensor(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1] * _aRes / 128.0f;
  dest1[1] = sum[2] * _aRes / 128.0f;
  dest1[2] = sum[3] * _aRes / 128.0f;
  dest2[0] = sum[4] * _gRes / 128.0f;
  dest2[1] = sum[5] * _gRes / 128.0f;
  dest2[2] = sum[6] * _gRes / 128.0f;

  if (dest1[0] > 0.8f)  {
    dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[0] < -0.8f) {
    dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[1] > 0.8f)  {
    dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[1] < -0.8f) {
    dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[2] > 0.8f)  {
    dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest1[2] < -0.8f) {
    dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}
/*Get Accel resolution*/
float ICM42605::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
  }
  return -1;
}
/*Get Gyro resolution*/
float ICM42605::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_15_125DPS:
      _gRes = 15.125f / 32768.0f;
      return _gRes;
      break;
    case GFS_31_25DPS:
      _gRes = 31.25f / 32768.0f;
      return _gRes;
      break;
    case GFS_62_5DPS:
      _gRes = 62.5f / 32768.0f;
      return _gRes;
      break;
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
      break;
  }
  return -1;
}

/*Reads the default data, accel, gyro & temp*/
void ICM42605::readSensor(int16_t * destination)
{
    readFall(ICM42605_TEMP_DATA1,13);
    //Data conversion
    destination[0] = ((int16_t)_buffer[0] << 8) | _buffer[1] ;      //Temperature
    destination[1] = ((int16_t)_buffer[2] << 8) | _buffer[3] ;      //Accel X
    destination[2] = ((int16_t)_buffer[4] << 8) | _buffer[5] ;      //Accel Y
    destination[3] = ((int16_t)_buffer[6] << 8) | _buffer[7] ;      //Accel Z
    destination[4] = ((int16_t)_buffer[8] << 8) | _buffer[9] ;      //Gyro X
    destination[5] = ((int16_t)_buffer[10] << 8) | _buffer[11] ;    //Gyro Y
    destination[6] = ((int16_t)_buffer[12] << 8) | _buffer[13] ;    //Gyro Z
    //Data saves in private data buffers

    //Temperature
    _t = ((float)destination[0] / 132.48) + 25; // (TEMP_DATA / 132.48) + 25 
    // Now we'll calculate the accleration value into actual g's
    _acc[0] = (float)destination[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    _acc[1] = (float)destination[2]*aRes - accelBias[1];   
    _acc[2] = (float)destination[3]*aRes - accelBias[2];  
    // Calculate the gyro value into actual degrees per second
    _gyro[0] = (float)destination[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    _gyro[1] = (float)destination[5]*gRes - gyroBias[1];  
    _gyro[2] = (float)destination[6]*gRes - gyroBias[2]; 

}
/* returns the accelerometer measurement in the x direction, m/s/s */
double ICM42605::getAccelX() {
  return _acc[0];
}
/* returns the accelerometer measurement in the y direction, m/s/s */
double ICM42605::getAccelY() {
  return _acc[1];
}
/* returns the accelerometer measurement in the z direction, m/s/s */
double ICM42605::getAccelZ() {
  return _acc[2];
}
/* returns the gyroscope measurement in the x direction, rad/s */
double ICM42605::getGyroX() {
  return _gyro[0];
}
/* returns the gyroscope measurement in the y direction, rad/s */
double ICM42605::getGyroY() {
  return _gyro[1];
}
/* returns the gyroscope measurement in the z direction, rad/s */
double ICM42605::getGyroZ() {
  return _gyro[2];
}
/* returns the die temperature, C */
double ICM42605::getTemperature_C() {
  return _t;
}
/*





*/
//Private
/*Writes on one Address*/
int ICM42605::writeAll(uint8_t subAddress, uint8_t data)
{
    //Initial conditions
    Serial.print("Writting to Address: ");
    Serial.print(subAddress, HEX);
    readAll(subAddress);
    Serial.print("Initial Data: ");
    Serial.print(_buffi, HEX);
    //Write
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));     // begin the transaction
    digitalWrite(_csPin,LOW);                                                   // select the ICM20689 chip
    _spi->transfer(subAddress);                                                 // write the register address
    _spi->transfer(data);                                                       // write the data
    digitalWrite(_csPin,HIGH);                                                  // deselect the ICM20689 chip
    _spi->endTransaction();                                                     // end the transaction
    Serial.print("Data sent: ");
    Serial.println(data, HEX);
    delay(10);
    //Validation
    readAll(subAddress);
    Serial.print("Data Recieved:  ");
    Serial.print(_buffi, HEX);
    Serial.println();
    if(_buffi == data) {
        return 1;
    }
    else{
        return -1;
    }
}
/*Writes a data(bit-wised) to ICM42605's register with a mask to specific bit's order*/
int ICM42605::writeBits(uint8_t subAddress, uint8_t data, uint8_t mask)
{  
    //Data conversion
    readAll(subAddress);
    uint8_t current_data= _buffi;
    uint8_t clear_data = current_data & ~mask;
    uint8_t new_value = clear_data | data;
    //Double validation
    if(writeAll(subAddress, new_value)<0){
        return -1;
    }
    else{
        return 1;
    }
}
/*Reads one Address*/
void ICM42605::readAll(uint8_t subAddress)
{
    Serial.print("Reading Address:    ");
    Serial.println(subAddress,HEX);
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK,MSBFIRST,SPI_MODE3));       // Begin the transaction
    digitalWrite(_csPin,LOW);                                                   // Select the ICM20689 chip 
    _spi->transfer(subAddress | SPI_READ);                                      // Specify the address and read mode 0x80
    _buffi = _spi->transfer(0x00);                                              // Saves read in _buffi  
    digitalWrite(_csPin,HIGH);                                                  // Deselect the ICM20689 chip
    _spi->endTransaction();                                                     // End the transaction                                                    
}
/*Reads a specific bit regisisters from ICM42605 given address*/
void ICM42605::readBits(uint8_t subAddress, uint8_t mask, uint8_t bitwised)
{
    readAll(subAddress);                    //Read all the data in subaddress and saves in _buffi
    //Data conversion
    byte data_received = _buffi;
    byte data_clear = data_received & mask;
    byte data_full = data_clear >> bitwised;
    _buffi = data_full;                    //Saves the data converted in _buffi
}
/*Reads in cascade from an initial Address*/
void ICM42605::readFall(uint8_t subAddress, uint8_t count)
{
    _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));     //Begin transaction in high speed
    digitalWrite(_csPin,LOW);                                                   // select the ICM20689 chip
    _spi->transfer(subAddress | SPI_READ);                                      // specify the starting register address
    for(uint8_t i = 0; i < count; i++){                                         // Cascade from initial address to count*
      _buffer[i] = _spi->transfer(0x00);                                        // Saves tead in _buffer[len(count)]
    }
    digitalWrite(_csPin,HIGH);                                                  // deselect the ICM20689 chip
    _spi->endTransaction();                                                     // end the transaction

}
/* gets the ICM20689 WHO_AM_I register value, expected to be 0x98 */
int ICM42605::whoAreWe()
{
    readAll(ICM42605_WHO_AM_I);
    Serial.print("Who Are We?  ");
    Serial.print(_buffi);
    Serial.println();
    return _buffi;
}
