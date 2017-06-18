/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The AMS CCS811 is an air quality sensor that provides equivalent CO2 and volatile organic measurements from direct
 *  I2C register reads as well as current and voltage (effective resistance of the sensing element). Gas sensors, including 
 *  this MEMs gas sensor in the CCS811 measure resistance of a substrate that changes when exposed to inert gasses and 
 *  volatile organic compounds. Changed in concentration vary exponentially with the changes in resistance. The CCS811
 *  has an embedded ASIC calibrated against most common indoor pollutants that returns a good estimate of
 *  equivalent CO2 concentration in parts per million (400 - 8192 range) and volatile organic componds in parts per billion (0 - 1187).
 *  The sensor is quite sensitive to breath and other human emissions.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "CCS811.h"

CCS811::CCS811(uint8_t intPin, uint8_t wakePin)
{
  pinMode(intPin, INPUT_PULLUP); // active LOW
  _intPin = intPin;
  pinMode(wakePin, OUTPUT);
  _wakePin = wakePin;   
}


uint8_t CCS811::getChipID()
{
 byte e = readByte(CCS811_ADDRESS, CCS811_ID);  // Read WHO_AM_I register for CCS8110
 return e;
}


void CCS811::checkCCS811Status() 
{
   // Check CCS811 status
  uint8_t status = readByte(CCS811_ADDRESS, CCS811_STATUS);
  Serial.print("status = 0X"); Serial.println(status, HEX);
  if(status & 0x80) {Serial.println("Firmware is in application mode. CCS811 is ready!");}
  else { Serial.println("Firmware is in boot mode!");}
  
  if(status & 0x10) {Serial.println("Valid application firmware loaded!");}
  else { Serial.println("No application firmware is loaded!");}

  if(status & 0x08) {Serial.println("New data available!");}
  else { Serial.println("No new data available!");}

  if(status & 0x01) {Serial.println("Error detected!");
        uint8_t error = readByte(CCS811_ADDRESS, CCS811_ERROR_ID);
        if(error & 0x01) Serial.println("CCS811 received invalid I2C write request!");
        if(error & 0x02) Serial.println("CCS811 received invalid I2C read request!");
        if(error & 0x04) Serial.println("CCS811 received unsupported mode request!");
        if(error & 0x08) Serial.println("Sensor resistance measurement at maximum range!");
        if(error & 0x10) Serial.println("Heater current is not in range!");
        if(error & 0x20) Serial.println("Heater voltage is not being applied correctly!");
  }
  else { Serial.println("No error detected!");
  }
  
  Serial.println(" ");
  
  }


  void CCS811::CCS811init(uint8_t AQRate)
  {
    // initialize CCS811 and check version and status
  byte HWVersion = readByte(CCS811_ADDRESS, CCS811_HW_VERSION);
  Serial.print("CCS811 Hardware Version = 0x"); Serial.println(HWVersion, HEX); 

  uint8_t FWBootVersion[2] = {0, 0}, FWAppVersion[2] = {0,0};
  readBytes(CCS811_ADDRESS, CCS811_FW_BOOT_VERSION, 2, &FWBootVersion[0]);
  Serial.println("CCS811 Firmware Boot Version: "); 
  Serial.print("Major = "); Serial.println((FWBootVersion[0] & 0xF0) >> 4); 
  Serial.print("Minor = "); Serial.println(FWBootVersion[0] & 0x04); 
  Serial.print("Trivial = "); Serial.println(FWBootVersion[1]); 
  
  readBytes(CCS811_ADDRESS, CCS811_FW_APP_VERSION, 2, &FWAppVersion[0]);
  Serial.println("CCS811 Firmware App Version: "); 
  Serial.print("Major = "); Serial.println((FWAppVersion[0] & 0xF0) >> 4); 
  Serial.print("Minor = "); Serial.println(FWAppVersion[0] & 0x04); 
  Serial.print("Trivial = "); Serial.println(FWAppVersion[1]); 

  // Check CCS811 status
  checkCCS811Status();
        uint8_t temp[1] = {0};
        temp[0] = CCS811_APP_START;
        Wire.transfer(CCS811_ADDRESS, &temp[0], 1, NULL, 0); 
        delay(100);
  checkCCS811Status();

  // set CCS811 measurement mode
  writeByte(CCS811_ADDRESS, CCS811_MEAS_MODE, AQRate << 4 | 0x08); // pulsed heating mode, enable interrupt
  uint8_t measmode = readByte(CCS811_ADDRESS, CCS811_MEAS_MODE);
  Serial.print("Confirm measurement mode = 0x"); Serial.println(measmode, HEX);
  }


  void CCS811::compensateCCS811(uint32_t compHumidity, uint32_t compTemp)
  {
      // Update CCS811 humidity and temperature compensation
      uint8_t temp[5] = {0, 0, 0, 0, 0};
      temp[0] = CCS811_ENV_DATA;
      temp[1] = ((compHumidity % 1024) / 100) > 7 ? (compHumidity/1024 + 1)<<1 : (compHumidity/1024)<<1;
      temp[2] = 0;
      if(((compHumidity % 1024) / 100) > 2 && (((compHumidity % 1024) / 100) < 8))
      {
       temp[1] |= 1;
      }

      compTemp += 2500;
      temp[3] = ((compTemp % 100) / 100) > 7 ? (compTemp/100 + 1)<<1 : (compTemp/100)<<1;
      temp[4] = 0;
      if(((compTemp % 100) / 100) > 2 && (((compTemp % 100) / 100) < 8))
      {
       temp[3] |= 1;
      }

      Wire.transfer(CCS811_ADDRESS, &temp[0], 5, NULL, 0);

  }

   void CCS811::readCCS811Data(uint8_t * destination)
   {
      uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      uint8_t status = readByte(CCS811_ADDRESS, CCS811_STATUS);
      
      if(status & 0x01) { // check for errors
        uint8_t error = readByte(CCS811_ADDRESS, CCS811_ERROR_ID);
        if(error & 0x01) Serial.println("CCS811 received invalid I2C write request!");
        if(error & 0x02) Serial.println("CCS811 received invalid I2C read request!");
        if(error & 0x04) Serial.println("CCS811 received unsupported mode request!");
        if(error & 0x08) Serial.println("Sensor resistance measurement at maximum range!");
        if(error & 0x10) Serial.println("Heater current is not in range!");
        if(error & 0x20) Serial.println("Heater voltage is not being applied correctly!");
      }

      readBytes(CCS811_ADDRESS, CCS811_ALG_RESULT_DATA, 8, &rawData[0]);

   for(int ii = 0; ii < 8; ii++)
   {
    destination[ii] = rawData[ii];
   }
   
   }


  
  // simple function to scan for I2C devices on the bus
void CCS811::I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    error = Wire.transfer(address, NULL, 0, NULL, 0);

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// I2C read/write functions for the BMP280 sensors

        void CCS811::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t CCS811::readByte(uint8_t address, uint8_t subAddress) {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void CCS811::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }
