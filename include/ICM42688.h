#ifndef ICM42688_H
#define ICM42688_H

#include "Arduino.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

// User Bank 0
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_TMST_FSYNCH               0x2B
#define ICM42688_TMST_FSYNCL               0x2C
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_FIFO_COUNTH               0x2E
#define ICM42688_FIFO_COUNTL               0x2F
#define ICM42688_FIFO_DATA                 0x30
#define ICM42688_APEX_DATA0                0x31
#define ICM42688_APEX_DATA1                0x32
#define ICM42688_APEX_DATA2                0x33
#define ICM42688_APEX_DATA3                0x34
#define ICM42688_APEX_DATA4                0x35
#define ICM42688_APEX_DATA5                0x36
#define ICM42688_INT_STATUS2               0x37   
#define ICM42688_INT_STATUS3               0x38   
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_INTF_CONFIG1              0x4D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_TMST_CONFIG               0x54
#define ICM42688_APEX_CONFIG0              0x56
#define ICM42688_SMD_CONFIG                0x57
#define ICM42688_FIFO_CONFIG1              0x5F
#define ICM42688_FIFO_CONFIG2              0x60
#define ICM42688_FIFO_CONFIG3              0x61
#define ICM42688_FSYNC_CONFIG              0x62
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_CONFIG1               0x64
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_INT_SOURCE1               0x66
#define ICM42688_INT_SOURCE3               0x68
#define ICM42688_INT_SOURCE4               0x69
#define ICM42688_FIFO_LOST_PKT0            0x6C
#define ICM42688_FIFO_LOST_PKT1            0x6D
#define ICM42688_SELF_TEST_CONFIG          0x70
#define ICM42688_WHO_AM_I                  0x75 // should return 0x47
#define ICM42688_REG_BANK_SEL              0x76

// User Bank 1
#define ICM42688_SENSOR_CONFIG0            0x03
#define ICM42688_GYRO_CONFIG_STATIC2       0x0B
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C
#define ICM42688_GYRO_CONFIG_STATIC4       0x0D
#define ICM42688_GYRO_CONFIG_STATIC5       0x0E
#define ICM42688_GYRO_CONFIG_STATIC6       0x0F
#define ICM42688_GYRO_CONFIG_STATIC7       0x10
#define ICM42688_GYRO_CONFIG_STATIC8       0x11
#define ICM42688_GYRO_CONFIG_STATIC9       0x12
#define ICM42688_GYRO_CONFIG_STATIC10      0x13
#define ICM42688_XG_ST_DATA                0x5F
#define ICM42688_YG_ST_DATA                0x60
#define ICM42688_ZG_ST_DATA                0x61
#define ICM42688_TMSTAL0                   0x63
#define ICM42688_TMSTAL1                   0x64
#define ICM42688_TMSTAL2                   0x62
#define ICM42688_INTF_CONFIG4              0x7A
#define ICM42688_INTF_CONFIG5              0x7B
#define ICM42688_INTF_CONFIG6              0x7C

// User Bank 2
#define ICM42688_ACCEL_CONFIG_STATIC2      0x03
#define ICM42688_ACCEL_CONFIG_STATIC3      0x04
#define ICM42688_ACCEL_CONFIG_STATIC4      0x05
#define ICM42688_XA_ST_DATA                0x3B
#define ICM42688_YA_ST_DATA                0x3C
#define ICM42688_ZA_ST_DATA                0x3D

// User Bank 4
#define ICM42688_APEX_CONFIG1              0x40
#define ICM42688_APEX_CONFIG2              0x41
#define ICM42688_APEX_CONFIG3              0x42
#define ICM42688_APEX_CONFIG4              0x43
#define ICM42688_APEX_CONFIG5              0x44
#define ICM42688_APEX_CONFIG6              0x45
#define ICM42688_APEX_CONFIG7              0x46
#define ICM42688_APEX_CONFIG8              0x47
#define ICM42688_APEX_CONFIG9              0x48
#define ICM42688_ACCEL_WOM_X_THR           0x4A
#define ICM42688_ACCEL_WOM_Y_THR           0x4B
#define ICM42688_ACCEL_WOM_Z_THR           0x4C
#define ICM42688_INT_SOURCE6               0x4D
#define ICM42688_INT_SOURCE7               0x4E
#define ICM42688_INT_SOURCE8               0x4F
#define ICM42688_INT_SOURCE9               0x50
#define ICM42688_INT_SOURCE10              0x51
#define ICM42688_OFFSET_USER0              0x77
#define ICM42688_OFFSET_USER1              0x78
#define ICM42688_OFFSET_USER2              0x79
#define ICM42688_OFFSET_USER3              0x7A
#define ICM42688_OFFSET_USER4              0x7B
#define ICM42688_OFFSET_USER5              0x7C
#define ICM42688_OFFSET_USER6              0x7D
#define ICM42688_OFFSET_USER7              0x7E
#define ICM42688_OFFSET_USER8              0x7F

#define ICM42688_ADDRESS           0x68   // Address of ICM42688 accel/gyro when ADO = 0

#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00 // default

#define GFS_2000DPS   0x00   // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_50DPS  0x05
#define GFS_31_25DPS  0x06
#define GFS_15_625DPS 0x07

// Low Noise mode
#define AODR_32kHz    0x01   
#define AODR_16kHz    0x02
#define AODR_8kHz     0x03
#define AODR_4kHz     0x04
#define AODR_2kHz     0x05
#define AODR_1kHz     0x06  // default
//Low Noise or Low Power modes
#define AODR_500Hz    0x0F
#define AODR_200Hz    0x07
#define AODR_100Hz    0x08
#define AODR_50Hz     0x09
#define AODR_25Hz     0x0A
#define AODR_12_5Hz   0x0B
// Low Power mode
#define AODR_6_25Hz   0x0C  
#define AODR_3_125Hz  0x0D
#define AODR_1_5625Hz 0x0E

#define GODR_32kHz  0x01   
#define GODR_16kHz  0x02
#define GODR_8kHz   0x03
#define GODR_4kHz   0x04
#define GODR_2kHz   0x05
#define GODR_1kHz   0x06 // default
#define GODR_500Hz  0x0F
#define GODR_200Hz  0x07
#define GODR_100Hz  0x08
#define GODR_50Hz   0x09
#define GODR_25Hz   0x0A
#define GODR_12_5Hz 0x0B

#define CLOCK_SEL_PLL 0x01
#define PWR_RESET 0x80
#define SEN_ENABLE 0x0F

#define BANK0  0x00
#define BANK1  0x01
#define BANK2  0x02
#define BANK3  0x03
#define BANK4  0x04


class ICM42688{
  public:
    enum GyroRange
    {
      GYRO_RANGE_15_625DPS,
      GYRO_RANGE_31_25DPS,
      GYRO_RANGE_62_5DPS,
      GYRO_RANGE_125DPS,
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G
    };
    enum DLPWBandWith{
      DLPF_BANDWIDTH_4000HZ,
      DLPF_BANDWIDTH_170HZ,
      DLPF_BANDWIDTH_82HZ,
      DLPF_BANDWIDTH_40HZ,
      DLPF_BANDWIDTH_20HZ,
      DLPF_BANDWIDTH_10HZ,
      DLPF_BANDWIDTH_5HZ
    };
   
    ICM42688(TwoWire &bus,uint8_t address);
    ICM42688(SPIClass &bus,uint8_t csPin);
    int begin(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);
    int init_neu(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);
    int setAccelRange(AccelRange range);
    int setGyroRange(GyroRange range);
    int setFilters(bool gyroFilters, bool accFilters);
    int enableDataReadyInterrupt();
    int disableDataReadyInterrupt();
    uint8_t isInterrupted();
    int setUseSPIHS(bool useSPIHS);
    int readSensor();
    int readAcc(double* acc);
    int readGyro(double* gyro);
    int readAccGyro(double* accGyro);
    double getAccelX_mss();
    double getAccelY_mss();
    double getAccelZ_mss();
    double getGyroX_rads();
    double getGyroY_rads();
    double getGyroZ_rads();
    double getGyroX_dps();
    double getGyroY_dps();
    double getGyroZ_dps();
    double getTemperature_C();

    int calibrateGyro();
    double getGyroBiasX_rads();
    double getGyroBiasY_rads();
    double getGyroBiasZ_rads();
    void setGyroBiasX_rads(double bias);
    void setGyroBiasY_rads(double bias);
    void setGyroBiasZ_rads(double bias);
    int calibrateAccel();
    double getAccelBiasX_mss();
    double getAccelScaleFactorX();
    double getAccelBiasY_mss();
    double getAccelScaleFactorY();
    double getAccelBiasZ_mss();
    double getAccelScaleFactorZ();
    void setAccelCalX(double bias,double scaleFactor);
    void setAccelCalY(double bias,double scaleFactor);
    void setAccelCalZ(double bias,double scaleFactor);
    int setDlpfBandwidth(DLPWBandWith bw);
    void correctAccelData();
    void reset();
    void readData(int16_t * destination);
    void offsetBias(float * dest1, float * dest2);
    void selfTest(int16_t * accelDiff, int16_t * gyroDiff, float * ratio);
    void calibrateAll();
    float getGres(uint8_t Gscale);
    float getAres(uint8_t Ascale);
     // data counts
    int16_t _accCounts[3] = {};

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, 
      AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_250DPS, AODR = AODR_200Hz, GODR = GODR_200Hz;

  protected:
    // i2c
    uint8_t _address = 0;
    TwoWire *_i2c = {};
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes = 0; // number of bytes received from I2C
    // spi
    SPIClass *_spi = {};
    uint8_t _csPin = 0;
    bool _useSPI = false;
    bool _useSPIHS = false;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
    const uint32_t SPI_HS_CLOCK = 8000000; // 8 MHz
    // buffer for reading from sensor
    uint8_t _buffer[15] = {};
    // data counts
    int16_t _gyroCounts[3] = {};
    int16_t _tcounts = 0;
           // data buffer
    double _acc[3] = {};
    double _gyro[3] = {};

    double _t = 0.0;
    uint8_t _isInterrupted = 0;
    // scale factors
    double _accelScale = 0.0;
    double _gyroScale = 0.0;
    const double _tempScale = 333.87f;
    const double _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    // gyro bias estimation
    size_t _numSamples = 100;
    double _gyroBD[3] = {};
    double _gyroB[3] = {};
    // accel bias and scale factor estimation
    double _accBD[3] = {};
    double _accB[3] = {};
    double _accS[3] = {1.0, 1.0, 1.0};
    double _accMax[3] = {};
    double _accMin[3] = {};
    // transformation matrix
    const int16_t tX[3] = {0,  1,  0};
    const int16_t tY[3] = {1,  0,  0};
    const int16_t tZ[3] = {0,  0, -1};
    // constants
    const double G = 9.807f;
    const double _d2r = 3.14159265359f/180.0f;
    const double _r2d = 180.0f/3.14159265359f;
    // ICM42688 registers
    // BANK 0
    const uint8_t ACCEL_OUT = 0x1F;
    const uint8_t GYRO_OUT = 0x25;
    const uint8_t TEMP_OUT = 0x1D;

    const uint8_t ACCEL_FS_SEL_2G = 0x80; // TODO: 0x60 in datasheet
    const uint8_t ACCEL_FS_SEL_4G = 0x60; // TODO: 0x40 in datasheet
    const uint8_t ACCEL_FS_SEL_8G = 0x40; // TODO: 0x20 in datasheet
    const uint8_t ACCEL_FS_SEL_16G = 0x20; // TODO: 0x00 in datasheet
    const uint8_t ACCEL_ODR_32KHZ = 0x01;
    const uint8_t ACCEL_ODR_16KHZ = 0x02;
    const uint8_t ACCEL_ODR_8KHZ = 0x03;
    const uint8_t ACCEL_ODR_4KHZ = 0x04;
    const uint8_t ACCEL_ODR_2KHZ = 0x05;
    const uint8_t ACCEL_ODR_1KHZ = 0x06;
    const uint8_t ACCEL_ODR_200HZ = 0x07;
    const uint8_t ACCEL_ODR_100HZ = 0x08;
    const uint8_t ACCEL_ODR_50HZ = 0x09;
    const uint8_t ACCEL_ODR_25HZ = 0x0A;
    const uint8_t ACCEL_ODR_12_5HZ = 0x0B;
    const uint8_t ACCEL_ODR_6_25HZ = 0x0C;
    const uint8_t ACCEL_ODR_3_125HZ = 0x0D;
    const uint8_t ACCEL_ODR_1_5625HZ = 0x0E;
    const uint8_t ACCEL_ODR_500HZ = 0x0F;

    const uint8_t GYRO_FS_SEL_15_625DPS = 0xE0;
    const uint8_t GYRO_FS_SEL_31_25DPS = 0xC0;
    const uint8_t GYRO_FS_SEL_62_5DPS = 0xA0;
    const uint8_t GYRO_FS_SEL_125DPS = 0x80;
    const uint8_t GYRO_FS_SEL_250DPS = 0x60;
    const uint8_t GYRO_FS_SEL_500DPS = 0x40;
    const uint8_t GYRO_FS_SEL_1000DPS = 0x20;
    const uint8_t GYRO_FS_SEL_2000DPS = 0x00;
    const uint8_t GYRO_ODR_32KHZ = 0x01;
    const uint8_t GYRO_ODR_16KHZ = 0x02;
    const uint8_t GYRO_ODR_8KHZ = 0x03;
    const uint8_t GYRO_ODR_4KHZ = 0x04;
    const uint8_t GYRO_ODR_2KHZ = 0x05;
    const uint8_t GYRO_ODR_1KHZ = 0x06;
    const uint8_t GYRO_ODR_200HZ = 0x07;
    const uint8_t GYRO_ODR_100HZ = 0x08;
    const uint8_t GYRO_ODR_50HZ = 0x09;
    const uint8_t GYRO_ODR_25HZ = 0x0A;
    const uint8_t GYRO_ODR_12_5HZ = 0x0B;
    const uint8_t GYRO_ODR_500HZ = 0x0F;

    const uint8_t GYRO_CONFIG1 = 0x51;
    const uint8_t TEMP_FILT_BW_4000HZ = 0b00000000;
    const uint8_t TEMP_FILT_BW_170HZ = 0b00100000;
    const uint8_t TEMP_FILT_BW_82HZ = 0b01000000;
    const uint8_t TEMP_FILT_BW_40HZ = 0b01100000;
    const uint8_t TEMP_FILT_BW_20HZ = 0b10000000;
    const uint8_t TEMP_FILT_BW_10HZ = 0b10100000;
    const uint8_t TEMP_FILT_BW_5HZ = 0b11000000;

    const uint8_t INT_CONFIG = 0x14;
    const uint8_t INT_HOLD_ANY = 0x08;
    const uint8_t INT_PULSE_100us = 0x03;
    const uint8_t INT_SOURCE0 = 0x65;
    const uint8_t RESET_DONE_INT1_EN = 0x10;
    const uint8_t UI_DRDY_INT1_EN = 0x10;
    const uint8_t INT_STATUS = 0x2D;



    const uint8_t WHO_AM_I = 0x75;
    const uint8_t FIFO_EN = 0x23;
    const uint8_t FIFO_TEMP_EN = 0x04;
    const uint8_t FIFO_GYRO = 0x02;
    const uint8_t FIFO_ACCEL = 0x01;
    const uint8_t FIFO_COUNT = 0x2E;
    const uint8_t FIFO_DATA = 0x30;


    // BANK 1
    const uint8_t GYRO_NF_ENABLE = 0x00;
    const uint8_t GYRO_NF_DISABLE = 0x01;
    const uint8_t GYRO_AAF_ENABLE = 0x00;
    const uint8_t GYRO_AAF_DISABLE = 0x02;

    // BANK 2
    const uint8_t ACCEL_AAF_ENABLE = 0x00;
    const uint8_t ACCEL_AAF_DISABLE = 0x01;


    // BANK 4
    const uint8_t ACCEL_OFFSET_Z = 0x7E;
    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
    float aRes, gRes;                                                        // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0.0f, 0.0f,0.0f}, gyroBias[3] = {0.0f, 0.0f,0.0f}; // offset biases for the accel and gyro
int16_t accelDiff[3] = {0, 0, 0}, gyroDiff[3] = {0, 0, 0};               // difference betwee ST and normal values
float  STratio[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};          // self-test results for the accel and gyro
float _aRes, _gRes;
};

class ICM42688_FIFO: public ICM42688 {
  public:
    using ICM42688::ICM42688;
    int enableFifo(bool accel,bool gyro,bool temp);
    int readFifo();
    void getFifoAccelX_mss(size_t *size,double* data);
    void getFifoAccelY_mss(size_t *size,double* data);
    void getFifoAccelZ_mss(size_t *size,double* data);
    void getFifoGyroX_rads(size_t *size,double* data);
    void getFifoGyroY_rads(size_t *size,double* data);
    void getFifoGyroZ_rads(size_t *size,double* data);
    void getFifoTemperature_C(size_t *size,double* data);
    void selfTest(int16_t * accelDiff, int16_t * gyroDiff, float * ratio);
  protected:
    // fifo
    bool _enFifoAccel = false;
    bool _enFifoGyro = false;
    bool _enFifoTemp = false;
    size_t _fifoSize = 0;
    size_t _fifoFrameSize = 0;
    double _axFifo[85] = {};
    double _ayFifo[85] = {};
    double _azFifo[85] = {};
    size_t _aSize = 0;
    double _gxFifo[85] = {};
    double _gyFifo[85] = {};
    double _gzFifo[85] = {};
    size_t _gSize = 0;
    double _tFifo[256] = {};
    size_t _tSize = 0;
};

#endif // ICM42688_H
