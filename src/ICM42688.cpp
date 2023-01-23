#include "Arduino.h"
#include "ICM42688.h"

/* ICM42688 object, input the I2C bus and address */
ICM42688::ICM42688(TwoWire &bus,uint8_t address) {
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
}


/* starts communication with the ICM42688 */
int ICM42688::begin(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR) {
  
    // starting the I2C bus
    _i2c->begin();
    // setting the I2C clock
    _i2c->setClock(_i2cRate);
  
  reset();  // software reset ICM42688 to default registers

   // set sensor resolutions for self test
   aRes = 4.0f/32768.0f;
   gRes = 250.0f/32768.0f;

   selfTest(accelDiff, gyroDiff, STratio);
   Serial.println("Accel Self Test:");
   Serial.print("Ax diff: "); Serial.print(accelDiff[0]* aRes * 1000.0f); Serial.println(" mg");
   Serial.print("Ay diff: "); Serial.print(accelDiff[1]* aRes * 1000.0f); Serial.println(" mg");
   Serial.print("Az diff: "); Serial.print(accelDiff[2]* aRes * 1000.0f); Serial.println(" mg");
   Serial.println("Should be between 50 and 1200 mg");

   Serial.println("Gyro Self Test:");
   Serial.print("Gx diff: "); Serial.print(gyroDiff[0] * gRes); Serial.println(" dps");
   Serial.print("Gy diff: "); Serial.print(gyroDiff[1] * gRes); Serial.println(" dps");
   Serial.print("Gz diff: "); Serial.print(gyroDiff[2] * gRes); Serial.println(" dps");
   Serial.println("Should be > 60 dps");
 
   Serial.print("Ax ratio: "); Serial.print(STratio[1]*100.0f, 0); Serial.println(" %");
   Serial.print("Ay ratio: "); Serial.print(STratio[2]*100.0f, 0); Serial.println(" %");
   Serial.print("Az ratio: "); Serial.print(STratio[3]*100.0f, 0); Serial.println(" %");
   Serial.println("Should be between 50 and 150%");

   Serial.println("Gyro Self Test:");
   Serial.print("Gx ratio: "); Serial.print(STratio[4]*100.0f, 0); Serial.println(" %");
   Serial.print("Gy ratio: "); Serial.print(STratio[5]*100.0f, 0); Serial.println(" %");
   Serial.print("Gz ratio: "); Serial.print(STratio[6]*100.0f, 0); Serial.println(" %");
   Serial.println("Should be between 50 and 150%");
   delay(2000);
      
   // get sensor resolutions for user settings, only need to do this once
   aRes = getAres(Ascale);
   gRes = getGres(Gscale);

  
   init(Ascale, Gscale, AODR, GODR);

   Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
   delay(4000);

   offsetBias(accelBias, gyroBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
   delay(1000); 

  return 1;
}

void ICM42688::selfTest(int16_t * accelDiff, int16_t * gyroDiff, float * ratio){
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelSTest[3] = {0, 0, 0}, gyroSTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

   // set sensor resolutions for self test
   aRes = 4.0f/32768.0f;
   gRes = 250.0f/32768.0f;
  writeRegister(ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
 
  writeRegister(ICM42688_PWR_MGMT0,  0x0F); // turn on accel and gyro in low noise mode
  delay(1);

  writeRegister(ICM42688_ACCEL_CONFIG0, AFS_4G << 5 | AODR_1kHz);  // FS = 2
  
  writeRegister(ICM42688_GYRO_CONFIG0,  GFS_250DPS << 5 | GODR_1kHz);  // FS = 3
 
  writeRegister(ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10

  readData(temp);
  accelNom[0] = temp[1];
  accelNom[1] = temp[2];
  accelNom[2] = temp[3];
  gyroNom[0]  = temp[4];
  gyroNom[1]  = temp[5];
  gyroNom[2]  = temp[6];
  
  writeRegister(ICM42688_SELF_TEST_CONFIG, 0x78); // accel self test
  delay(100); // let accel respond
  readData(temp);
  accelSTest[0] = temp[1];
  accelSTest[1] = temp[2];
  accelSTest[2] = temp[3];

  writeRegister(ICM42688_SELF_TEST_CONFIG, 0x07); // gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroSTest[0] = temp[4];
  gyroSTest[1] = temp[5];
  gyroSTest[2] = temp[6];

  writeRegister(ICM42688_SELF_TEST_CONFIG, 0x00); // normal mode

  accelDiff[0] = accelSTest[0] - accelNom[0];
  if(accelDiff[0] < 0) accelDiff[0] *= -1;        // make sure difference values are positive
  accelDiff[1] = accelSTest[1] - accelNom[1];
  if(accelDiff[1] < 0)accelDiff[1] *= -1;
  accelDiff[2] = accelSTest[2] - accelNom[2];
  if(accelDiff[2] < 0) accelDiff[2] *= -1;
  gyroDiff[0] = gyroSTest[0] - gyroNom[0];
  if(gyroDiff[0] < 0) gyroDiff[0] *= -1;
  gyroDiff[1] = gyroSTest[1] - gyroNom[1];
  if(gyroDiff[1] < 0) gyroDiff[1] *= -1;
  gyroDiff[2] = gyroSTest[2] - gyroNom[2];
  if(gyroDiff[2] < 0) gyroDiff[2] *= -1;
  
  writeRegister(ICM42688_REG_BANK_SEL, 0x01); // select register bank 1
  readRegisters(ICM42688_XG_ST_DATA,1,_buffer);// gyro self-test output generated during manufacturing tests
  temp[4] = _buffer[0]; 
  readRegisters(ICM42688_YG_ST_DATA,1,_buffer);
  temp[5] = _buffer[0]; 
  readRegisters(ICM42688_ZG_ST_DATA,1,_buffer);
  temp[6] = _buffer[0]; 

  writeRegister(ICM42688_REG_BANK_SEL, 0x02); // select register bank 2

  readRegisters(ICM42688_XA_ST_DATA,1,_buffer);  // accel self-test output generated during manufacturing tests
  temp[1] = _buffer[0]; 
  readRegisters(ICM42688_YA_ST_DATA,1,_buffer);
  temp[2] = _buffer[0]; 
  readRegisters(ICM42688_ZA_ST_DATA,1,_buffer);
  temp[3] = _buffer[0]; 

  ratio[1] = accelDiff[0] / (1310.0f * powf(1.01f, temp[1] - 1) + 0.5f);
  ratio[2] = accelDiff[1] / (1310.0f * powf(1.01f, temp[2] - 1) + 0.5f);
  ratio[3] = accelDiff[2] / (1310.0f * powf(1.01f, temp[3] - 1) + 0.5f);
  ratio[4] = gyroDiff[0]  / (2620.0f * powf(1.01f, temp[4] - 1) + 0.5f);
  ratio[5] = gyroDiff[1] /  (2620.0f * powf(1.01f, temp[5] - 1) + 0.5f);
  ratio[6] = gyroDiff[2] /  (2620.0f * powf(1.01f, temp[6] - 1) + 0.5f);
  
  writeRegister(ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


/* reads the most current data from ICM42688 and stores in buffer */
int ICM42688::readSensor() {
  // grab the data from the ICM42688
 /* if (readRegisters(TEMP_OUT, 14, _buffer) < 0) {
    return -1;
  }*/
 readData(ICM42688Data); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)ICM42688Data[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set in g
     ay = (float)ICM42688Data[2]*aRes - accelBias[1];   
     az = (float)ICM42688Data[3]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gx = (float)ICM42688Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set in deg/s
     gy = (float)ICM42688Data[5]*gRes - gyroBias[1];  
     gz = (float)ICM42688Data[6]*gRes - gyroBias[2]; 
  return 1;
}



/* writes a byte to ICM42688 register given a register address and data */
int ICM42688::writeRegister(uint8_t subAddress, uint8_t data) {
  /* write data to device */

    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // write the register address
    _i2c->write(data); // write the data
    _i2c->endTransmission();
  

  delay(10);

  /* read back the register */
  readRegisters(subAddress,1,_buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

/* reads registers from ICM42688 given a starting register address, number of bytes, and a pointer to store data */
int ICM42688::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {

    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
    if (_numBytes == count) {
      for(uint8_t i = 0; i < count; i++) {
        dest[i] = _i2c->read();
      }
      return 1;
    } else {
      return -1;
    }
  
}


void ICM42688::offsetBias(float * dest1, float * dest2){
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};
    
  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1]*_aRes/128.0f;
  dest1[1] = sum[2]*_aRes/128.0f;
  dest1[2] = sum[3]*_aRes/128.0f;
  dest2[0] = sum[4]*_gRes/128.0f;
  dest2[1] = sum[5]*_gRes/128.0f;
  dest2[2] = sum[6]*_gRes/128.0f;

  if(dest1[0] > 0.8f)  {dest1[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest1[0] < -0.8f) {dest1[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest1[1] > 0.8f)  {dest1[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest1[1] < -0.8f) {dest1[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest1[2] > 0.8f)  {dest1[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
  if(dest1[2] < -0.8f) {dest1[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation

  // load offset biases into offset registers (optional, comment out if not desired)
  temp[0] = (int16_t) (-dest1[0] / 0.00048828125f); // Ax 0.5 mg resolution
  temp[1] = (int16_t) (-dest1[1] / 0.00048828125f); // Ay
  temp[2] = (int16_t) (-dest1[2] / 0.00048828125f); // Az
  temp[3] = (int16_t) (-dest2[0] / 0.03125f);       // Gx 1/32 dps resolution
  temp[4] = (int16_t) (-dest2[1] / 0.03125f);       // Gy
  temp[5] = (int16_t) (-dest2[2] / 0.03125f);       // Gz

  writeRegister(ICM42688_REG_BANK_SEL, 0x04); // select register bank 4

  writeRegister(ICM42688_OFFSET_USER5,  temp[0] & 0x00FF); // lower Ax byte
  writeRegister(ICM42688_OFFSET_USER6,  temp[1] & 0x00FF); // lower Ay byte
  writeRegister(ICM42688_OFFSET_USER8,  temp[2] & 0x00FF); // lower Az byte
  writeRegister(ICM42688_OFFSET_USER2,  temp[4] & 0x00FF); // lower Gy byte
  writeRegister(ICM42688_OFFSET_USER3,  temp[5] & 0x00FF); // lower Gz byte
  writeRegister(ICM42688_OFFSET_USER0,  temp[3] & 0x00FF); // lower Gx byte
  writeRegister(ICM42688_OFFSET_USER4,  (temp[0] & 0x0F00) >> 4 | (temp[5] & 0x0F00) >> 8); // upper Ax and Gz bytes
  writeRegister(ICM42688_OFFSET_USER7,  (temp[2] & 0x0F00) >> 4 | (temp[1] & 0x0F00) >> 8); // upper Az and Ay bytes
  writeRegister(ICM42688_OFFSET_USER1,  (temp[4] & 0x0F00) >> 4 | (temp[3] & 0x0F00) >> 8); // upper Gy and Gx bytes
  
  writeRegister(ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  
}

void ICM42688::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readRegisters(ICM42688_TEMP_DATA1, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}

void ICM42688::reset()
{
  // reset device
  writeRegister(ICM42688_DEVICE_CONFIG,  0x01); // Set bit 0 to 1 to reset ICM42688
  delay(1); // Wait 1 ms for all registers to reset 
}

uint8_t ICM42688::getChipID()
{
  uint8_t temp;
  readRegisters( ICM42688_WHO_AM_I,1,&temp);
  return temp;
}

float ICM42688::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         return _aRes;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         return _aRes;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         return _aRes;
         break;
    case AFS_16G:
         _aRes = 16.0f/32768.0f;
         return _aRes;
         break;
  }
}

float ICM42688::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
     case GFS_15_625DPS:
          _gRes = 15.625f/32768.0f;
          return _gRes;
          break;
    case GFS_31_25DPS:
          _gRes = 31.25f/32768.0f;
          return _gRes;
          break;
    case GFS_62_50DPS:
          _gRes = 62.5f/32768.0f;
          return _gRes;
          break;
    case GFS_125DPS:
          _gRes = 125.0f/32768.0f;
          return _gRes;
          break;
    case GFS_250DPS:
          _gRes = 250.0f/32768.0f;
          return _gRes;
          break;
    case GFS_500DPS:
          _gRes = 500.0f/32768.0f;
          return _gRes;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0f/32768.0f;
         return _gRes;
         break;
    case GFS_2000DPS:
          _gRes = 2000.0f/32768.0f;
         return _gRes;
         break;
  }
}

uint8_t ICM42688::DRStatus()
{
  uint8_t temp;
  readRegisters( ICM42688_INT_STATUS,1,&temp); 
  return temp;
}


void ICM42688::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
  writeRegister( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0

  writeRegister( ICM42688_PWR_MGMT0,  0x0F); // turn on accel and gyro in low noise mode
  delay(1);

  writeRegister( ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
  
  writeRegister( ICM42688_GYRO_CONFIG0,  Gscale << 5 | GODR); // set gyro ODR and FS
  
  writeRegister( ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10
 
   // interrupt handling
/*  writeRegister( ICM42688_INT_CONFIG, 0x18 | 0x03 );      // push-pull, pulsed, active HIGH interrupts  
  uint8_t temp = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_INT_CONFIG1);     // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)
  writeRegister( ICM42688_INT_CONFIG1, temp & ~(0x10));   // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)
  writeRegister( ICM42688_INT_SOURCE0, 0x08);             // data ready interrupt routed to INT1
  */
  writeRegister( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


float ICM42688::getAccX(){
return ax;
}

float ICM42688::getAccY(){
return ay;
}

float ICM42688::getAccZ(){
return az;
}

float ICM42688::getGyroX(){
  return gx;
}

float ICM42688::getGyroY(){
  return gy;
}

float ICM42688::getGyroZ(){
  return gz;
}