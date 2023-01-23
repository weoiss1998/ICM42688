#include "Arduino.h"
#include "ICM42688.h"

/* ICM42688 object, input the I2C bus and address */
ICM42688::ICM42688(TwoWire &bus,uint8_t address) {
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
  _useSPI = false; // set to use I2C
}

/* ICM42688 object, input the SPI bus and chip select pin */
ICM42688::ICM42688(SPIClass &bus,uint8_t csPin) {
  _spi = &bus; // SPI bus
  _csPin = csPin; // chip select pin
  _useSPI = true; // set to use SPI
}

/* starts communication with the ICM42688 */
int ICM42688::begin(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR) {
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
  init_neu(Ascale, Gscale, AODR, GODR);

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

/* sets the accelerometer full scale range to values other than default */
int ICM42688::setAccelRange(AccelRange range) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(range) {
    case ACCEL_RANGE_2G: {
      // setting the accel range to 2G
      if(writeRegister(ICM42688_ACCEL_CONFIG0,ACCEL_FS_SEL_2G|ACCEL_ODR_100HZ) < 0) {
        return -1;
      }
      _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
      break;
    }
    case ACCEL_RANGE_4G: {
      // setting the accel range to 4G
      if(writeRegister(ICM42688_ACCEL_CONFIG0,ACCEL_FS_SEL_4G|ACCEL_ODR_100HZ) < 0) {
        return -1;
      }
      _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
      break;
    }
    case ACCEL_RANGE_8G: {
      // setting the accel range to 8G
      if(writeRegister(ICM42688_ACCEL_CONFIG0,ACCEL_FS_SEL_8G|ACCEL_ODR_100HZ) < 0) {
        return -1;
      }
      _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
      break;
    }
    case ACCEL_RANGE_16G: {
      // setting the accel range to 16G
      if(writeRegister(ICM42688_ACCEL_CONFIG0,ACCEL_FS_SEL_16G|ACCEL_ODR_100HZ) < 0) {
        return -1;
      }
      //_accelScale = G*2048.0f;
      _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G -> 0,004788647
      //calibrateAccel();
      break;
    }
  }
  _accelRange = range;
  return 1;
}

/* sets the gyro full scale range to values other than default */
int ICM42688::setGyroRange(GyroRange range) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(range) {
    case GYRO_RANGE_15_625DPS: {
      // setting the gyro range to 15.625DPS
      if(writeRegister(ICM42688_GYRO_CONFIG0,GYRO_FS_SEL_15_625DPS) < 0) {
        return -1;
      }
      _gyroScale = 15.625f/32767.5f * _d2r; // setting the gyro scale to 15.625DPS
      break;
    }
    case GYRO_RANGE_31_25DPS: {
      // setting the gyro range to 31.25DPS
      if(writeRegister(ICM42688_GYRO_CONFIG0,GYRO_FS_SEL_31_25DPS) < 0) {
        return -1;
      }
      _gyroScale = 31.25f/32767.5f * _d2r; // setting the gyro scale to 31.25DPS
      break;
    }
    case GYRO_RANGE_62_5DPS: {
      // setting the gyro range to 62.5DPS
      if(writeRegister(ICM42688_GYRO_CONFIG0,GYRO_FS_SEL_62_5DPS) < 0) {
        return -1;
      }
      _gyroScale = 62.5f/32767.5f * _d2r; // setting the gyro scale to 62.5DPS
      break;
    }
    case GYRO_RANGE_125DPS: {
      // setting the gyro range to 125DPS
      if(writeRegister(ICM42688_GYRO_CONFIG0,GYRO_FS_SEL_125DPS) < 0) {
        return -1;
      }
      _gyroScale = 125.0f/32767.5f * _d2r; // setting the gyro scale to 125DPS
      break;
    }
    case GYRO_RANGE_250DPS: {
      // setting the gyro range to 250DPS
      if(writeRegister(ICM42688_GYRO_CONFIG0,GYRO_FS_SEL_250DPS) < 0) {
        return -1;
      }
      _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
      break;
    }
    case GYRO_RANGE_500DPS: {
      // setting the gyro range to 500DPS
      if(writeRegister(ICM42688_GYRO_CONFIG0,GYRO_FS_SEL_500DPS) < 0) {
        return -1;
      }
      _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
      break;
    }
    case GYRO_RANGE_1000DPS: {
      // setting the gyro range to 1000DPS
      if(writeRegister(ICM42688_GYRO_CONFIG0,GYRO_FS_SEL_1000DPS) < 0) {
        return -1;
      }
      _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
      break;
    }
    case GYRO_RANGE_2000DPS: {
      // setting the gyro range to 2000DPS
      if(writeRegister(ICM42688_GYRO_CONFIG0,GYRO_FS_SEL_2000DPS) < 0) {
        return -1;
      }
      _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
      break;
    }
  }
  _gyroRange = range;
  return 1;
}
/* sets the tempeartue full scale bandwith to values other than default */
int ICM42688::setDlpfBandwidth(DLPWBandWith bw)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(bw) {
    case DLPF_BANDWIDTH_4000HZ: {
      // setting the bandwith of the temperature to 4000Hz
      if(writeRegister(GYRO_CONFIG1,TEMP_FILT_BW_4000HZ) < 0) {
        return -1;
      }
      break;
    }
    case DLPF_BANDWIDTH_170HZ: {
      // setting the bandwith of the temperature to 170Hz
      if(writeRegister(GYRO_CONFIG1,TEMP_FILT_BW_170HZ) < 0) {
        return -1;
      }
      break;
    }
    case DLPF_BANDWIDTH_82HZ: {
      // setting the bandwith of the temperature to 82Hz
      if(writeRegister(GYRO_CONFIG1,TEMP_FILT_BW_82HZ) < 0) {
        return -1;
      }
      break;
    }
    case DLPF_BANDWIDTH_40HZ: {
      // setting the bandwith of the temperature to 40Hz
      if(writeRegister(GYRO_CONFIG1,TEMP_FILT_BW_40HZ) < 0) {
        return -1;
      }
      break;
    }
    case DLPF_BANDWIDTH_20HZ: {
      // setting the bandwith of the temperature to 20Hz
      if(writeRegister(GYRO_CONFIG1,TEMP_FILT_BW_20HZ) < 0) {
        return -1;
      }
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      // setting the bandwith of the temperature to 10Hz
      if(writeRegister(GYRO_CONFIG1,TEMP_FILT_BW_10HZ) < 0) {
        return -1;
      }
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      // setting the bandwith of the temperature to 5Hz
      if(writeRegister(GYRO_CONFIG1,TEMP_FILT_BW_5HZ) < 0) {
        return -1;
      }
      break;
    }
  }
  return 1;

}

int ICM42688::setFilters(bool gyroFilters, bool accFilters) {
  if (writeRegister(ICM42688_REG_BANK_SEL , BANK1) < 0) {
    return -1;
  }
  if (gyroFilters == true) {
    if (writeRegister(ICM42688_GYRO_CONFIG_STATIC2 , GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0) {
      return -2;
    }
  }
  else {
    if (writeRegister(ICM42688_GYRO_CONFIG_STATIC2 , GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0) {
      return -3;
    }
  }
  if (writeRegister(ICM42688_REG_BANK_SEL, BANK2) < 0) {
    return -4;
  }
  if (accFilters == true) {
    if (writeRegister(ICM42688_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0) {
      return -5;
    }
  }
  else {
    if (writeRegister(ICM42688_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0) {
      return -6;
    }
  }
  if (writeRegister(ICM42688_REG_BANK_SEL, BANK0) < 0) {
    return -7;
  }
  return 1;
}

/* enables the data ready interrupt */
int ICM42688::enableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the interrupt */
  if (writeRegister(INT_CONFIG,INT_PULSE_100us) < 0) { // setup interrupt, pulse
  // if (writeRegister(INT_CONFIG,INT_HOLD_ANY) < 0) { // setup interrupt, hold, any read operation
    return -1;
  }
  if (writeRegister(INT_SOURCE0,UI_DRDY_INT1_EN) < 0) { // set to data ready
    return -2;
  }
  return 1;
}

/* disables the data ready interrupt */
int ICM42688::disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(INT_SOURCE0,RESET_DONE_INT1_EN) < 0) { // set to reset done (disable interrupt)
    return -1;
  }
  return 1;
}

/* disables the data ready interrupt */
uint8_t ICM42688::isInterrupted() {
  _useSPIHS = false; // use the high speed SPI for data readout
  readRegisters(INT_STATUS, 1, &_isInterrupted);
  return _isInterrupted & 0x08;
}

/* set SPI mode */
int ICM42688::setUseSPIHS(bool useSPIHS) {
  _useSPIHS = useSPIHS;
  return 1;
}

/* reads the most current data from ICM42688 and stores in buffer */
int ICM42688::readSensor() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM42688
  if (readRegisters(TEMP_OUT, 14, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _accCounts[0] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _accCounts[1] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _accCounts[2] = (((int16_t)_buffer[6]) << 8) | _buffer[7];
  _tcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _gyroCounts[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gyroCounts[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _gyroCounts[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];
 // _acc[0] = (((double)(tX[1]*_accCounts[1]) * _accelScale) - _accB[0])*_accS[0];
 // _acc[1] = (((double)(tY[0]*_accCounts[0]) * _accelScale) - _accB[1])*_accS[1];
 // _acc[2] = (((double)(tZ[2]*_accCounts[2]) * _accelScale) - _accB[2])*_accS[2];
   _t = ((((double) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
  /*  _acc[0] = (((double)(tX[1]*_accCounts[1]) * _accelScale));
  _acc[1] = (((double)(tY[0]*_accCounts[0]) * _accelScale));
  _acc[2] = (((double)(tZ[2]*_accCounts[2]) * _accelScale));
  _gyro[0] = ((double)(tX[1]*_gyroCounts[1]) * _gyroScale) - _gyroB[0];
  _gyro[1] = ((double)(tY[0]*_gyroCounts[0]) * _gyroScale) - _gyroB[1];
  _gyro[2] = ((double)(tZ[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[2];*/

  
  // Now we'll calculate the accleration value into actual g's
     _acc[0] = (float)_accCounts[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     _acc[1] = (float)_accCounts[1]*aRes - accelBias[1];   
     _acc[2] = (float)_accCounts[2]*aRes - accelBias[2]; 

    // Calculate the gyro value into actual degrees per second
     _gyro[0] = (float)_gyroCounts[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     _gyro[0] = (float)_gyroCounts[1]*gRes - gyroBias[1];  
     _gyro[0] = (float)_gyroCounts[2]*gRes - gyroBias[2]; 
  return 1;
}

/* reads the most current acc data from ICM42688 */
int ICM42688::readAcc(double* acc) {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM42688
  if (readRegisters(ACCEL_OUT, 6, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _accCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _accCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _accCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _acc[0] = (((double)(tX[1]*_accCounts[1]) * _accelScale) - _accB[0])*_accS[0];
  _acc[1] = (((double)(tY[0]*_accCounts[0]) * _accelScale) - _accB[1])*_accS[1];
  _acc[2] = (((double)(tZ[2]*_accCounts[2]) * _accelScale) - _accB[2])*_accS[2];
  /*
  // Now we'll calculate the accleration value into actual g's
     _acc[0] = (float)_accCounts[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     _acc[1] = (float)_accCounts[1]*aRes - accelBias[1];   
     _acc[2] = (float)_accCounts[2]*aRes - accelBias[2];  */
  memcpy(acc, _acc, 3*sizeof(double));
  return 1;
}

/* reads the most current gyro data from ICM42688 */
int ICM42688::readGyro(double* gyro) {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM42688
  if (readRegisters(GYRO_OUT, 6, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _gyroCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _gyroCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _gyroCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _gyro[0] = ((double)(tX[1]*_gyroCounts[1]) * _gyroScale) - _gyroB[0];
  _gyro[1] = ((double)(tY[0]*_gyroCounts[0]) * _gyroScale) - _gyroB[1];
  _gyro[2] = ((double)(tZ[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[2];
/*
     // Calculate the gyro value into actual degrees per second
     _gyro[0] = (float)_gyroCounts[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     _gyro[0] = (float)_gyroCounts[1]*gRes - gyroBias[1];  
     _gyro[0] = (float)_gyroCounts[2]*gRes - gyroBias[2]; */
  memcpy(gyro, _gyro, 3*sizeof(double));
  return 1;
}

/* reads the most current accGyro data from ICM42688 */
int ICM42688::readAccGyro(double* accGyro) {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM42688
  if (readRegisters(ACCEL_OUT, 12, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _accCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _accCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _accCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _gyroCounts[0] = (((int16_t)_buffer[6]) << 8) | _buffer[7];
  _gyroCounts[1] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gyroCounts[2] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _acc[0] = (((double)(tX[1]*_accCounts[1]) * _accelScale) - _accB[0])*_accS[0];
  _acc[1] = (((double)(tY[0]*_accCounts[0]) * _accelScale) - _accB[1])*_accS[1];
  _acc[2] = (((double)(tZ[2]*_accCounts[2]) * _accelScale) - _accB[2])*_accS[2];
  _gyro[0] = ((double)(tX[1]*_gyroCounts[1]) * _gyroScale) - _gyroB[0];
  _gyro[1] = ((double)(tY[0]*_gyroCounts[0]) * _gyroScale) - _gyroB[1];
  _gyro[2] = ((double)(tZ[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[2];
/*
  // Now we'll calculate the accleration value into actual g's
     _acc[0] = (float)_accCounts[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     _acc[1] = (float)_accCounts[1]*aRes - accelBias[1];   
     _acc[2] = (float)_accCounts[2]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     _gyro[0] = (float)_gyroCounts[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     _gyro[0] = (float)_gyroCounts[1]*gRes - gyroBias[1];  
     _gyro[0] = (float)_gyroCounts[2]*gRes - gyroBias[2]; */
  memcpy(&accGyro[0], _acc, 3*sizeof(double));
  memcpy(&accGyro[3], _gyro, 3*sizeof(double));
  return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
double ICM42688::getAccelX_mss() {
  return _acc[0];
}

/* returns the accelerometer measurement in the y direction, m/s/s */
double ICM42688::getAccelY_mss() {
  return _acc[1];
}

/* returns the accelerometer measurement in the z direction, m/s/s */
double ICM42688::getAccelZ_mss() {
  return _acc[2];
}

/* returns the gyroscope measurement in the x direction, rad/s */
double ICM42688::getGyroX_rads() {
  return _gyro[0];
}

/* returns the gyroscope measurement in the y direction, rad/s */
double ICM42688::getGyroY_rads() {
  return _gyro[1];
}

/* returns the gyroscope measurement in the z direction, rad/s */
double ICM42688::getGyroZ_rads() {
  return _gyro[2];
}

/* returns the gyroscope measurement in the x direction, rad/s */
double ICM42688::getGyroX_dps() {
  return _gyro[0]*_r2d;
}

/* returns the gyroscope measurement in the y direction, rad/s */
double ICM42688::getGyroY_dps() {
  return _gyro[1]*_r2d;
}

/* returns the gyroscope measurement in the z direction, rad/s */
double ICM42688::getGyroZ_dps() {
  return _gyro[2]*_r2d;
}

/* returns the die temperature, C */
double ICM42688::getTemperature_C() {
  return _t;
}

/* configures and enables the FIFO buffer  */
int ICM42688_FIFO::enableFifo(bool accel,bool gyro,bool temp) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(temp*FIFO_TEMP_EN)) < 0) {
    return -2;
  }
  _enFifoAccel = accel;
  _enFifoGyro = gyro;
  _enFifoTemp = temp;
  _fifoFrameSize = accel*6 + gyro*6 + temp*2;
  return 1;
}


/* estimates the gyro biases */
int ICM42688::calibrateGyro() {
  // set the range, bandwidth, and srd
  if (setGyroRange(GYRO_RANGE_250DPS) < 0) {
    return -1;
  }
  // take samples and find bias
  _gyroBD[0] = 0;
  _gyroBD[1] = 0;
  _gyroBD[2] = 0;
  for (size_t i=0; i < _numSamples; i++) {
    readSensor();
    _gyroBD[0] += (getGyroX_rads() + _gyroB[0])/((double)_numSamples);
    _gyroBD[1] += (getGyroY_rads() + _gyroB[1])/((double)_numSamples);
    _gyroBD[2] += (getGyroZ_rads() + _gyroB[2])/((double)_numSamples);
    delay(20);
  }
  _gyroB[0] = (double)_gyroBD[0];
  _gyroB[1] = (double)_gyroBD[1];
  _gyroB[2] = (double)_gyroBD[2];

  // set the range, bandwidth, and srd back to what they were
  if (setGyroRange(_gyroRange) < 0) {
    return -4;
  }
  return 1;
}

/* returns the gyro bias in the X direction, rad/s */
double ICM42688::getGyroBiasX_rads() {
  return _gyroB[0];
}

/* returns the gyro bias in the Y direction, rad/s */
double ICM42688::getGyroBiasY_rads() {
  return _gyroB[1];
}

/* returns the gyro bias in the Z direction, rad/s */
double ICM42688::getGyroBiasZ_rads() {
  return _gyroB[2];
}

/* sets the gyro bias in the X direction to bias, rad/s */
void ICM42688::setGyroBiasX_rads(double bias) {
  _gyroB[0] = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void ICM42688::setGyroBiasY_rads(double bias) {
  _gyroB[1] = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void ICM42688::setGyroBiasZ_rads(double bias) {
  _gyroB[2] = bias;
}
void ICM42688::correctAccelData(){
   /*if (writeRegister(BANK_SEL, BANK4)<0){
    readSensor();
    delay(1);
    float temp = _acc[2]+G;
    temp=temp/G;
    temp=temp*2000.0;
    uint16_t senden= abs(temp);
    uint8_t upper_bits = highByte(senden);
    upper_bits=upper_bits<<4;
    upper_bits=upper_bits && 0b01110000;
    uint8_t lower_bits=lowByte(senden);
    if(temp<0){
      upper_bits=upper_bits||0b10000000;
    }
    writeRegister(ACCEL_OFFSET_Z,upper_bits);
    delay(1);
    writeRegister((ACCEL_OFFSET_Z+1),lower_bits);
    delay(1);
    writeRegister(BANK_SEL, BANK0);
    delay(1);
   }*/


}
/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int ICM42688::calibrateAccel() {
  // set the range, bandwidth, and srd
  if (setAccelRange(ACCEL_RANGE_2G) < 0) {
    return -1;
  }
  // take samples and find min / max
  _accBD[0] = 0;
  _accBD[1] = 0;
  _accBD[2] = 0;
  for (size_t i=0; i < _numSamples; i++) {
    readSensor();
    _accBD[0] += (getAccelX_mss()/_accS[0] + _accB[0])/((double)_numSamples);
    _accBD[1] += (getAccelY_mss()/_accS[1] + _accB[1])/((double)_numSamples);
    _accBD[2] += (getAccelZ_mss()/_accS[2] + _accB[2])/((double)_numSamples);
    delay(20);
  }
  if (_accBD[0] > 9.0f) {
    _accMax[0] = (double)_accBD[0];
  }
  if (_accBD[1] > 9.0f) {
    _accMax[1] = (double)_accBD[1];
  }
  if (_accBD[2] > 9.0f) {
    _accMax[2] = (double)_accBD[2];
  }
  if (_accBD[0] < -9.0f) {
    _accMin[0] = (double)_accBD[0];
  }
  if (_accBD[1] < -9.0f) {
    _accMin[1] = (double)_accBD[1];
  }
  if (_accBD[2] < -9.0f) {
    _accMin[2] = (double)_accBD[2];
  }

  // find bias and scale factor
  if ((abs(_accMin[0]) > 9.0f) && (abs(_accMax[0]) > 9.0f)) {
    _accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
    _accS[0] = G/((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
  }
  if ((abs(_accMin[1]) > 9.0f) && (abs(_accMax[1]) > 9.0f)) {
    _accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
    _accS[1] = G/((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
  }
  if ((abs(_accMin[2]) > 9.0f) && (abs(_accMax[2]) > 9.0f)) {
    _accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
    _accS[2] = G/((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
  }

  // set the range, bandwidth, and srd back to what they were
  if (setAccelRange(_accelRange) < 0) {
    return -4;
  }
  //correctAccelData();
  return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
double ICM42688::getAccelBiasX_mss() {
  return _accB[0];
}

/* returns the accelerometer scale factor in the X direction */
double ICM42688::getAccelScaleFactorX() {
  return _accS[0];
}

/* returns the accelerometer bias in the Y direction, m/s/s */
double ICM42688::getAccelBiasY_mss() {
  return _accB[1];
}

/* returns the accelerometer scale factor in the Y direction */
double ICM42688::getAccelScaleFactorY() {
  return _accS[1];
}

/* returns the accelerometer bias in the Z direction, m/s/s */
double ICM42688::getAccelBiasZ_mss() {
  return _accB[2];
}

/* returns the accelerometer scale factor in the Z direction */
double ICM42688::getAccelScaleFactorZ() {
  return _accS[2];
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void ICM42688::setAccelCalX(double bias,double scaleFactor) {
  _accB[0] = bias;
  _accS[0] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void ICM42688::setAccelCalY(double bias,double scaleFactor) {
  _accB[1] = bias;
  _accS[1] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void ICM42688::setAccelCalZ(double bias,double scaleFactor) {
  _accB[2] = bias;
  _accS[2] = scaleFactor;
}



/* writes a byte to ICM42688 register given a register address and data */
int ICM42688::writeRegister(uint8_t subAddress, uint8_t data) {
  /* write data to device */
  if( _useSPI ) {
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1)); // begin the transaction
    digitalWrite(_csPin,LOW); // select the ICM42688 chip
    _spi->transfer(subAddress); // write the register address
    _spi->transfer(data); // write the data
    digitalWrite(_csPin,HIGH); // deselect the ICM42688 chip
    _spi->endTransaction(); // end the transaction
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
  if( _useSPI ) {
    // begin the transaction
    if(_useSPIHS) {
      _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE1));
    }
    else{
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE1));
    }
    digitalWrite(_csPin,LOW); // select the ICM42688 chip
    _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    for(uint8_t i = 0; i < count; i++) {
      dest[i] = _spi->transfer(0x00); // read the data
    }
    digitalWrite(_csPin,HIGH); // deselect the ICM42688 chip
    _spi->endTransaction(); // end the transaction
    return 1;
  }
  else{
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
}

/* gets the ICM42688 WHO_AM_I register value, expected to be 0x98 */
int ICM42688::whoAmI() {
  // read the WHO AM I register
  if (readRegisters(WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
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

  writeRegister(ICM42688_REG_BANK_SEL, BANK4); // select register bank 4

  writeRegister(ICM42688_OFFSET_USER5,  temp[0] & 0x00FF); // lower Ax byte
  writeRegister(ICM42688_OFFSET_USER6,  temp[1] & 0x00FF); // lower Ay byte
  writeRegister(ICM42688_OFFSET_USER8,  temp[2] & 0x00FF); // lower Az byte
  writeRegister(ICM42688_OFFSET_USER2,  temp[4] & 0x00FF); // lower Gy byte
  writeRegister(ICM42688_OFFSET_USER3,  temp[5] & 0x00FF); // lower Gz byte
  writeRegister(ICM42688_OFFSET_USER0,  temp[3] & 0x00FF); // lower Gx byte
  writeRegister(ICM42688_OFFSET_USER4,  (temp[0] & 0x0F00) >> 4 | (temp[5] & 0x0F00) >> 8); // upper Ax and Gz bytes
  writeRegister(ICM42688_OFFSET_USER7,  (temp[2] & 0x0F00) >> 4 | (temp[1] & 0x0F00) >> 8); // upper Az and Ay bytes
  writeRegister(ICM42688_OFFSET_USER1,  (temp[4] & 0x0F00) >> 4 | (temp[3] & 0x0F00) >> 8); // upper Gy and Gx bytes
  
  writeRegister(ICM42688_REG_BANK_SEL, BANK0); // select register bank 0
  
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

void ICM42688::calibrateAll(){
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
   delay(500);
      

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

int ICM42688::init_neu(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
  writeRegister(ICM42688_REG_BANK_SEL, BANK0); // select register bank 0
  delay(1);
     // select clock source to gyro
  if(writeRegister(ICM42688_INTF_CONFIG1 ,CLOCK_SEL_PLL) < 0) {
    return -1;
  }
  // reset the ICM42688
  writeRegister(ICM42688_DEVICE_CONFIG ,PWR_RESET);
  // wait for ICM42688 to come back up
  delay(1);
  // select clock source to gyro
  if(writeRegister(ICM42688_INTF_CONFIG1,CLOCK_SEL_PLL) < 0) {
    return -2;
  }

    // check the WHO AM I byte, expected value is 0x47 (decimal 71)
  if(whoAmI() != 71) {
    return -3;
  }
  // enable accelerometer and gyro
  if(writeRegister(ICM42688_PWR_MGMT0 ,SEN_ENABLE) < 0) {// turn on accel and gyro in low noise mode
    return -4;
  }
  delay(1);
calibrateAll();
  // setting accel range to 16G and 32kHz as default
  if(writeRegister( ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR) < 0) {// set accel ODR and FS
    return -5;
  }
  
    if(writeRegister( ICM42688_GYRO_CONFIG0,  Gscale << 5 | GODR) < 0) {// set gyro ODR and FS
    return -6;
  }
  
  writeRegister( ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10
 
 delay(2000);
   // interrupt handling
/*  writeRegister( ICM42688_INT_CONFIG, 0x18 | 0x03 );      // push-pull, pulsed, active HIGH interrupts  
  uint8_t temp = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_INT_CONFIG1);     // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)
  writeRegister( ICM42688_INT_CONFIG1, temp & ~(0x10));   // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)
  writeRegister( ICM42688_INT_SOURCE0, 0x08);             // data ready interrupt routed to INT1
  */

     // get sensor resolutions for user settings, only need to do this once
   aRes = getAres(Ascale);
   gRes = getGres(Gscale);
  

  offsetBias(accelBias, gyroBias);
  delay(500); 

  // // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
  // if (setFilters(false, false) < 0) {
  //   return -7;
  // }
/*
  // estimate gyro bias
  if (calibrateGyro() < 0) {
    return -8;
  }*/
  // successful init, return 1
  writeRegister( ICM42688_REG_BANK_SEL, BANK0); // select register bank 0
  return 1;
}

/* reads data from the ICM42688 FIFO and stores in buffer */
int ICM42688_FIFO::readFifo() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // get the fifo size
  readRegisters(FIFO_COUNT, 2, _buffer);
  _fifoSize = (((uint16_t) (_buffer[0]&0x0F)) <<8) + (((uint16_t) _buffer[1]));
  // read and parse the buffer
  for (size_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
    // grab the data from the ICM42688
    if (readRegisters(FIFO_DATA,_fifoFrameSize,_buffer) < 0) {
      return -1;
    }
    if (_enFifoAccel) {
      // combine into 16 bit values
      _accCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
      _accCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
      _accCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
      // transform and convert to double values
      _axFifo[i] = (((double)(tX[0]*_accCounts[0] + tX[1]*_accCounts[1] + tX[2]*_accCounts[2]) * _accelScale)-_accB[0])*_accS[0];
      _ayFifo[i] = (((double)(tY[0]*_accCounts[0] + tY[1]*_accCounts[1] + tY[2]*_accCounts[2]) * _accelScale)-_accB[1])*_accS[1];
      _azFifo[i] = (((double)(tZ[0]*_accCounts[0] + tZ[1]*_accCounts[1] + tZ[2]*_accCounts[2]) * _accelScale)-_accB[2])*_accS[2];
      _aSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoTemp) {
      // combine into 16 bit values
      _tcounts = (((int16_t)_buffer[0 + _enFifoAccel*6]) << 8) | _buffer[1 + _enFifoAccel*6];
      // transform and convert to double values
      _tFifo[i] = ((((double) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
      _tSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoGyro) {
      // combine into 16 bit values
      _gyroCounts[0] = (((int16_t)_buffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
      _gyroCounts[1] = (((int16_t)_buffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
      _gyroCounts[2] = (((int16_t)_buffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
      // transform and convert to double values
      _gxFifo[i] = ((double)(tX[0]*_gyroCounts[0] + tX[1]*_gyroCounts[1] + tX[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[0];
      _gyFifo[i] = ((double)(tY[0]*_gyroCounts[0] + tY[1]*_gyroCounts[1] + tY[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[1];
      _gzFifo[i] = ((double)(tZ[0]*_gyroCounts[0] + tZ[1]*_gyroCounts[1] + tZ[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[2];
      _gSize = _fifoSize/_fifoFrameSize;
    }
  }
  return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void ICM42688_FIFO::getFifoAccelX_mss(size_t *size,double* data) {
  *size = _aSize;
  memcpy(data,_axFifo,_aSize*sizeof(double));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void ICM42688_FIFO::getFifoAccelY_mss(size_t *size,double* data) {
  *size = _aSize;
  memcpy(data,_ayFifo,_aSize*sizeof(double));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void ICM42688_FIFO::getFifoAccelZ_mss(size_t *size,double* data) {
  *size = _aSize;
  memcpy(data,_azFifo,_aSize*sizeof(double));
}

/* returns the gyroscope FIFO size and data in the x direction, rad/s */
void ICM42688_FIFO::getFifoGyroX_rads(size_t *size,double* data) {
  *size = _gSize;
  memcpy(data,_gxFifo,_gSize*sizeof(double));
}

/* returns the gyroscope FIFO size and data in the y direction, rad/s */
void ICM42688_FIFO::getFifoGyroY_rads(size_t *size,double* data) {
  *size = _gSize;
  memcpy(data,_gyFifo,_gSize*sizeof(double));
}

/* returns the gyroscope FIFO size and data in the z direction, rad/s */
void ICM42688_FIFO::getFifoGyroZ_rads(size_t *size,double* data) {
  *size = _gSize;
  memcpy(data,_gzFifo,_gSize*sizeof(double));
}

/* returns the die temperature FIFO size and data, C */
void ICM42688_FIFO::getFifoTemperature_C(size_t *size,double* data) {
  *size = _tSize;
  memcpy(data,_tFifo,_tSize*sizeof(double));
}
