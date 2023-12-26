/* *****************************************************************************
   File:     ICM42605
   Project:  Smart Bedding
   Author:   GIBIC Medtech
   Version:  V_1.0
 *                                                                             *
   Created on September 14, 2023
 * *****************************************************************************/

#include "ICM42605.h"

/* ICM20689 object, input the I2C bus and address */
ICM42605::ICM42605(TwoWire &bus,uint8_t address){
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
  _useSPI = false; // set to use I2C
}

/* ICM20689 object, input the SPI bus and chip select pin */
ICM42605::ICM42605(SPIClass &bus,uint8_t csPin, uint8_t APEX){
  _spi = &bus; // SPI bus
  _csPin = csPin; // chip select pin
  _useSPI = true; // set to use SPI
  if(APEX==1){
    _useAPEX = true;
  }
  if(APEX==2){
    _useTAP = true;
  }
}

int ICM42605::begin(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
  aRes = getAres(Ascale);
  gRes = getGres(Gscale);

  if( _useSPI ) { // using SPI for communication
    // use low speed SPI for register setting
    _useSPIHS = false;
    // setting CS pin to output
    pinMode(_csPin,OUTPUT);
    // setting CS pin high
    digitalWrite(_csPin,HIGH);
    // begin SPI communication
    _spi->begin();
  } else { // using I2C for communication
    // starting the I2C bus
    _i2c->begin();
    // setting the I2C clock
    _i2c->setClock(_i2cRate);
  }

  reset();

  // check the WHO AM I byte, expected value is 0x42 (decimal 66)
  if(whoAmI() != 66) {
    return -1;
  }
  // enable gyro and accel in low noise mode
  if(writeRegister(ICM42605_PWR_MGMT0,0x0F) < 0) { 
    return -2;
  }
   // gyro full scale and data rate
  if(writeRegister(ICM42605_GYRO_CONFIG0, GODR | Gscale << 5 ) < 0) { 
    return -3;
  }
  // set accel full scale and data rate
  if(writeRegister(ICM42605_ACCEL_CONFIG0, AODR | Ascale << 5) < 0) {
    return -4;
  }
  // set temperature sensor low pass filter to 5Hz, use first order gyro filter
  if(writeRegister(ICM42605_GYRO_CONFIG1,0xD0) < 0) {
    return -5;
  }

  offsetBias(accelBias, gyroBias);
  // Set apex, but only step counter... ill be in the future the other features
  if(_useAPEX){
    writeRegister(ICM42605_SIGNAL_PATH_RESET, 0X01<<5);
    delay(1);
    writeRegister(ICM42605_APEX_CONFIG1, 0X0A<<4);
    writeRegister(ICM42605_APEX_CONFIG2, 0X08 <<4 |0X05);
    writeRegister(ICM42605_APEX_CONFIG3, 0X02 <<5 |0X04 <<2 |0X01);
    if(writeRegister(ICM42605_APEX_CONFIG9, 0X00)<0){
      return -10;
    }
    writeRegister(ICM42605_SIGNAL_PATH_RESET, 0X01<<6);
    delay(50);
    writeRegister(ICM42605_INT_SOURCE6, 0X01<<5);
    writeRegister(ICM42605_APEX_CONFIG0, 0X01<<5);
  }


  //Set tap Deteccion 
  if(_useTAP){
    //1,2,3 
    // tap max 2
    // tap min 3   0x1x2
    // rap tav 3



  }



  return 1;
}

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


/* Read apex measurement */
int ICM42605::readApex(int16_t * destination) 
{
  if(_useAPEX){
    _useSPIHS=true;
    if(readRegisters(ICM42605_APEX_DATA0, 2, _buffer)<0){
      return -1;
    }
    destination[0]=((int16_t)_buffer[0] << 8) | _buffer[1] ;
    destination[1]=((int16_t)_buffer[2]);
    //debe haber alguna parte de procesamiento?
    _step[0]= (float)destination[0]; //Step count so far 16bits...
    _step[1]= (float)destination[1]; // cadence
    return 1;
  }else{
    return -1;
  }
}
/* returns the APEX measurement for step count */
double ICM42605::getStepCount(){
  return _step[0];
}
/* returns the APEX measurement for step cadence */
double ICM42605::getCadence(){
  return _step[1];
}


int ICM42605::readSensor(int16_t * destination)
{
  _useSPIHS = true; // use the high speed SPI for data readout
  if (readRegisters(ICM42605_TEMP_DATA1, 15, _buffer) < 0) { // se puede reduci el 15 a 13
    return -1;
  }
  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)_buffer[0] << 8) | _buffer[1] ; //Temperature
  destination[1] = ((int16_t)_buffer[2] << 8) | _buffer[3] ; //Accel X
  destination[2] = ((int16_t)_buffer[4] << 8) | _buffer[5] ; //Accel Y
  destination[3] = ((int16_t)_buffer[6] << 8) | _buffer[7] ; //Accel Z
  destination[4] = ((int16_t)_buffer[8] << 8) | _buffer[9] ; //Gyro X
  destination[5] = ((int16_t)_buffer[10] << 8) | _buffer[11] ; //Gyro Y
  destination[6] = ((int16_t)_buffer[12] << 8) | _buffer[13] ; //Gyro Z

  _t = ((float)destination[0] / 132.48) + 25; // (TEMP_DATA / 132.48) + 25 

// Now we'll calculate the accleration value into actual g's
  _acc[0] = (float)destination[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
  _acc[1] = (float)destination[2]*aRes - accelBias[1];   
  _acc[2] = (float)destination[3]*aRes - accelBias[2];  

// Calculate the gyro value into actual degrees per second
  _gyro[0] = (float)destination[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
  _gyro[1] = (float)destination[5]*gRes - gyroBias[1];  
  _gyro[2] = (float)destination[6]*gRes - gyroBias[2]; 

  return 1;
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

void ICM42605::reset()
{
  // reset device
  writeRegister(ICM42605_DEVICE_CONFIG,0x01); // reset the ICM42605
  delay(1); // wait for ICM42605 to come back up
}

uint8_t ICM42605::status()
{
  // reset device
  uint8_t temp;
  readRegisters(ICM42605_ADDRESS, ICM42605_INT_STATUS, &temp);
  return temp;
}

//Data igual forma Shift hex<<position
int ICM42605::write2(uint8_t subAddress, uint8_t data, bool fulx, byte mask){
  Serial.print("Writting to Address:   ");
  Serial.println(subAddress,HEX);
  readRegisters(subAddress,1,_buffer);

  if(_useSPI){
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    digitalWrite(_csPin,LOW);// select the ICM20689 chip
    if (fulx)
    {
      _spi->transfer(subAddress); // write the register address
      _spi->transfer(data); // write the data
    }
    else
    {
      _spi->transfer(subAddress | SPI_READ);
      byte current_data_value = _buffer[0]; //adress value
      byte clear_address = current_data_value & ~mask; //Clear the current data with the desired positions(mask)
      _newValue = clear_address | data;
      _spi->transfer(subAddress); // write the register address
      _spi->transfer(_newValue); // write the data
    }
    digitalWrite(_csPin,HIGH); // deselect the ICM20689 chip
    _spi->endTransaction(); // end the transaction
  }

  //Validation
  readRegisters(subAddress,1,_buffer);
  if(fulx){
    if(_buffer[0] == data) {
      return 1;
    }
    else{
      return -1;
  }

  }else{
    if(_buffer[0] == _newValue) {
      return 1;
    }
    else{
      return -1;
    }
  }
}

/* writes a byte to ICM20689 register given a register address and data */
int ICM42605::writeRegister(uint8_t subAddress, uint8_t data){
  /* write data to device */
  Serial.print("Writting to Address: ");
  Serial.println(subAddress, HEX);
  if( _useSPI ){
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    digitalWrite(_csPin,LOW); // select the ICM20689 chip
    _spi->transfer(subAddress); // write the register address
    _spi->transfer(data); // write the data
    digitalWrite(_csPin,HIGH); // deselect the ICM20689 chip
    _spi->endTransaction(); // end the transaction
    Serial.print("Data sent: ");
    Serial.println(data, HEX);
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // write the register address
    _i2c->write(data); // write the data
    _i2c->endTransmission();
  }

  delay(10);

  /* read back the register */
  readRegisters(subAddress,1,_buffer);
  Serial.print("Data Recieved: ");
  int count = sizeof(_buffer) / sizeof(_buffer[0]);
  for (int i = 0; i < count; i++) {
    Serial.print(_buffer[i], HEX);
  }
  Serial.print(" - First char: ");
  Serial.print(_buffer[0], HEX);
  Serial.println();
  Serial.println();
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

/* reads registers from ICM20689 given a starting register address, number of bytes, and a pointer to store data */
int ICM42605::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
  if( _useSPI ){
    // begin the transaction
    if(_useSPIHS){
      _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    else{
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    digitalWrite(_csPin,LOW); // select the ICM20689 chip
    _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    for(uint8_t i = 0; i < count; i++){
      dest[i] = _spi->transfer(0x00); // read the data
    }
    digitalWrite(_csPin,HIGH); // deselect the ICM20689 chip
    _spi->endTransaction(); // end the transaction
    return 1;
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
    if (_numBytes == count) {
      for(uint8_t i = 0; i < count; i++){
        dest[i] = _i2c->read();
      }
      return 1;
    } else {
      return -1;
    }
  }
}

/* gets the ICM20689 WHO_AM_I register value, expected to be 0x98 */
int ICM42605::whoAmI(){
  // read the WHO AM I register
  if (readRegisters(ICM42605_WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  Serial.print("Who Am I?: ");
  int count = sizeof(_buffer) / sizeof(_buffer[0]);
  for (int i = 0; i < count; i++) {
    Serial.print(_buffer[i]);
  }
  Serial.print(" - First char: ");
  Serial.print(_buffer[0]);
  Serial.println();
  Serial.println();
  return _buffer[0];
}
