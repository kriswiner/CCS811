#ifndef CCS811_h
#define CCS811_h

#include "Arduino.h"
#include "Wire.h"

/* CCS811 Registers
http://www.mouser.com/ds/2/588/CCS811_DS000459_3-00-1098798.pdf
*/
#define CCS811_STATUS             0x00
#define CCS811_MEAS_MODE          0x01
#define CCS811_ALG_RESULT_DATA    0x02
#define CCS811_RAW_DATA           0x03
#define CCS811_ENV_DATA           0x05
#define CCS811_NTC                0x06
#define CCS811_THRESHOLDS         0x10
#define CCS811_BASELINE           0x11
#define CCS811_HW_ID              0x20  // WHO_AM_I should be 0x81
#define CCS811_ID                 0x20  // WHO_AM_I should be 0x1X
#define CCS811_HW_VERSION         0x21  
#define CCS811_FW_BOOT_VERSION    0x23
#define CCS811_FW_APP_VERSION     0x24
#define CCS811_ERROR_ID           0xE0
#define CCS811_APP_START          0xF4
#define CCS811_SW_RESET           0xFF

#define CCS811_ADDRESS            0x5A   // Address of the CCS811 Air Quality Sensor

#define  dt_idle  0x00
#define  dt_1sec  0x01
#define  dt_10sec 0x02
#define  dt_60sec 0x03

class CCS811
{
  public: 
  CCS811(uint8_t intPin, uint8_t wakePin);
  void checkCCS811Status();
  void CCS811init(uint8_t AQRate);
  uint8_t getChipID();
  void compensateCCS811(uint32_t compHumidity, uint32_t compTemp);
  void readCCS811Data(uint8_t * destination);
  void I2Cscan();
  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
  private:
  uint8_t _intPin;
  uint8_t _wakePin;
};

#endif
