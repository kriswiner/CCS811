/* BME280_CCS811_Butterfly.ino
 by: Kris Winer
 date: March 25, 2017
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 The Bosch BME280 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 mode with power consumption of 20 microAmp, or in a lower-resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.

 The AMS CCS811 is an air quality sensor that provides equivalent CO2 and volatile organic measurements from direct
 I2C register reads as well as current and voltage (effective resistance of the sensing element). Gas sensors, including 
 this MEMs gas sensor in the CCS811 measure resistance of a substrate that changes when exposed to inert gasses and 
 volatile organic compounds. Changed in concentration vary exponentially with the changes in resistance. The CCS811
 has an embedded ASIC calibrated against most common indoor pollutants that returns a good estimate of
 equivalent CO2 concentration in parts per million (400 - 8192 range) and volatile organic componds in parts per billion (0 - 1187).
 The sensor is quite sensitive to breath and other human emissions.
 
 This sketch was written to operate the air quality sensor with the Butterfly STM32L433 Development Board using Thomas Roell's 
 STM32L4 Arduino Core. If you are using a different MCU you will have to modify the sketch since the Wire.transfer functions
 and RTC.h library are specific to the STM32L4.
 
 SDA and SCL have 4K7 pull-up resistors (to 3.3V).
 
 Butterfly hardware setup:
 SDA ----------------------- 21
 SCL ----------------------- 22
 
  */
#include "Wire.h"   
#include <RTC.h>

// BME280 registers  https://www.bosch-sensortec.com/bst/products/all_products/bme280
#define BME280_HUM_LSB    0xFE
#define BME280_HUM_MSB    0xFD
#define BME280_TEMP_XLSB  0xFC
#define BME280_TEMP_LSB   0xFB
#define BME280_TEMP_MSB   0xFA
#define BME280_PRESS_XLSB 0xF9
#define BME280_PRESS_LSB  0xF8
#define BME280_PRESS_MSB  0xF7
#define BME280_CONFIG     0xF5
#define BME280_CTRL_MEAS  0xF4
#define BME280_STATUS     0xF3
#define BME280_CTRL_HUM   0xF2
#define BME280_RESET      0xE0
#define BME280_ID         0xD0  // should be 0x60
#define BME280_CALIB00    0x88
#define BME280_CALIB26    0xE1

// CCS811 Registers http://ams.com/eng/Products/Environmental-Sensors/Gas-Sensors/CCS811
#define CCS811_STATUS             0x00
#define CCS811_MEAS_MODE          0x01
#define CCS811_ALG_RESULT_DATA    0x02
#define CCS811_RAW_DATA           0x03
#define CCS811_ENV_DATA           0x05
#define CCS811_NTC                0x06
#define CCS811_THRESHOLDS         0x10
#define CCS811_BASELINE           0x11
#define CCS811_HW_ID              0x20  // WHO_AM_I should be 0x81
#define CCS811_ID                 0x20   
#define CCS811_HW_VERSION         0x21  
#define CCS811_FW_BOOT_VERSION    0x23
#define CCS811_FW_APP_VERSION     0x24
#define CCS811_ERROR_ID           0xE0
#define CCS811_APP_START          0xF4
#define CCS811_SW_RESET           0xFF

#define BME280_ADDRESS            0x76   // Address of BMP280 altimeter when BDO = LOW (default)
#define CCS811_ADDRESS            0x5A   // Address of the CCS811 Air Quality Sensor when CDO = LOW (default)
//#define BME280_ADDRESS            0x77   // Address of BMP280 altimeter when BDO = HIGH
//#define CCS811_ADDRESS            0x5B   // Address of the CCS811 Air Quality Sensor when CDO = HIGH

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

enum Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum Hosr {
  H_OSR_00 = 0,  // no op
  H_OSR_01,
  H_OSR_02,
  H_OSR_04,
  H_OSR_08,
  H_OSR_16
};

enum Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode {
  BME280Sleep = 0,
  forced,
  forced2,
  normal
};

enum SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_10ms,
  t_20ms
};

enum AQRate {  // specify  frequency of air quality measurement
  dt_idle = 0,
  dt_1sec = 1,
  dt_10sec,
  dt_60sec
};

// Specify CCS811 rate
uint8_t AQRate = dt_10sec;

// Specify BME280 configuration
uint8_t Posr = P_OSR_16, Hosr = H_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_021ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate
// t_fine carries fine temperature as global value for BME280
int32_t t_fine;

float Temperature, Pressure, Humidity; // stores BME280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp, compTemp;    // pressure, humidity, and temperature raw count output for BME280
uint32_t compHumidity, compPress;
int16_t rawHumidity;  // variables to hold raw BME280 humidity value

// BME280 compensation parameters
uint8_t dig_H1, dig_H3, dig_H6;
uint16_t dig_T1, dig_P1, dig_H4, dig_H5;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2;

float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

uint32_t delt_t = 0, count = 0, sumCount = 0, slpcnt = 0;  // used to control display output rate

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 16;
const uint8_t hours = 9;

/* Change these values to set the current initial date */
const byte day = 10;
const byte month = 1;
const byte year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;
float VDDA, VBAT;
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// CCS811
const byte CCS811Interrupt = 8;
const byte CCS811Enable    = 9;
const byte VbatMon = A4;

bool newCCS811Data  = false;

void setup()
{
  Serial.begin(38400);
  delay(4000);
  Serial.println("Serial enabled!");

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led on, active LOW on Butterfly

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs on Butterfly

  // Configure interrupts
  pinMode(CCS811Interrupt, INPUT); // active LOW

  // Configure CCS811 enable
  pinMode(CCS811Enable, OUTPUT);
  digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor
 
  Wire.begin(TWI_PINS_20_21); // set master mode using pins 20 (SDA) and 21 (SCL) 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  I2Cscan(); // should detect BME280 at 0x76 and CCS811 at 0x5A

  digitalWrite(CCS811Enable, HIGH); // set LOW to enable the CCS811 air quality sensor

  delay(1000);

  // Set the time
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // Set the date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);

  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte f = readByte(BME280_ADDRESS, BME280_ID);  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(f, HEX); Serial.print(" I should be ");Serial.println(0x60, HEX);
  delay(1000); 

  // Read the WHO_AM_I register of the CCS811 this is a good test of communication
  digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor
  byte g = readByte(CCS811_ADDRESS, CCS811_ID);  // Read WHO_AM_I register for CCS8110
  digitalWrite(CCS811Enable, HIGH); // set HIGH to disable the CCS811 air quality sensor
  Serial.print("CCS811 "); Serial.print("I AM "); Serial.print(g, HEX); Serial.print(" I should be "); Serial.println(0x81, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(f == 0x60 && g == 0x81) {

  Serial.println("CCS811+BME280 are online..."); Serial.println(" ");
   
    writeByte(BME280_ADDRESS, BME280_RESET, 0xB6); // reset BME280 before initialization
    delay(100);

    BME280Init(); // Initialize BME280 altimeter

    // initialize CCS811 and check version and status
    digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811init();
    digitalWrite(CCS811Enable, HIGH); // set HIGH to disable the CCS811 air quality sensor

    attachInterrupt(CCS811Interrupt,  myinthandler2, FALLING); // enable CCS811 interrupt
    }
    else 
    if(f != 0x60) Serial.println(" BME280 not functioning!");    
    if(g != 0x81) Serial.println(" CCS811 not functioning!");
}

void loop()
{
    // BME280 Data
    rawTemp =   readBME280Temperature();
    compTemp = BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f; // temperature in Centigrade
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
    rawPress =  readBME280Pressure();
    pressure = (float) BME280_compensate_P(rawPress)/25600.0f; // Pressure in millibar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));
    rawHumidity =  readBME280Humidity();
    compHumidity = BME280_compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH

      Serial.println("BME280:");
      Serial.print("Altimeter temperature = "); 
      Serial.print( temperature_C, 2); 
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = "); 
      Serial.print(temperature_F, 2); 
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); 
      Serial.print(pressure, 2);  
      Serial.println(" mbar");// pressure in millibar
      Serial.print("Altitude = "); 
      Serial.print(altitude, 2); 
      Serial.println(" feet");
      Serial.print("Altimeter humidity = "); 
      Serial.print(humidity, 1);  
      Serial.println(" %RH");// pressure in millibar
      Serial.println(" ");

      // CCS811 data 
      Serial.println("CCS811:");
            
      // If intPin goes LOW, all data registers have new data
      if(newCCS811Data == true) {  // On interrupt, read data
      newCCS811Data = false;  // reset newData flag

      // Update CCS811 humidity and temperature compensation using latest BME280 dta
      CCS811Compensate();
     
      digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor
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
          
      eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
      TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
      Current = (rawData[6] & 0xFC) >> 2;
      Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f), 3; 
     } 
     
      digitalWrite(CCS811Enable, HIGH); // set LOW to enable the CCS811 air quality sensor 

      Serial.print("Eq CO2 in ppm = "); Serial.println(eCO2);
      Serial.print("TVOC in ppb = "); Serial.println(TVOC);
      Serial.print("Sensor current (uA) = "); Serial.println(Current);
      Serial.print("Sensor voltage (V) = "); Serial.println(Voltage, 2);  
      Serial.println(" ");
      
      // Read RTC
      Serial.println("RTC:");
      Day = RTC.getDay();
      Month = RTC.getMonth();
      Year = RTC.getYear();
      Seconds = RTC.getSeconds();
      Minutes = RTC.getMinutes();
      Hours   = RTC.getHours();     
      if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
      Serial.print(":"); 
      if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
      Serial.print(":"); 
      if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

      Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
      Serial.println(" ");

      VBAT = (127.0f/100.0f) * 3.30f * ((float)analogRead(VbatMon))/4095.0f;
      Serial.print("VBAT = "); Serial.println(VBAT, 2); 


      digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH);    // toggle led for 10 ms

      STM32.sleep();    // time out in sleep mode to save power
       
      delay(990);

}

//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler2()
{
  newCCS811Data = true;
}


  int32_t readBME280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME280_ADDRESS, BME280_TEMP_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 24 | (uint32_t) rawData[1] << 16 | (uint32_t) rawData[2] << 8) >> 12);
}

int32_t readBME280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME280_ADDRESS, BME280_PRESS_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 24 | (uint32_t) rawData[1] << 16 | (uint32_t) rawData[2] << 8) >> 12);
}

int32_t readBME280Humidity()
{
  uint8_t rawData[2];  // 16-bit humidity register data stored here
  readBytes(BME280_ADDRESS, BME280_HUM_MSB, 2, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 24 | (uint32_t) rawData[1] << 16) ) >> 16;
}


void BME280Init()
{
  // Configure the BME280
  // Set H oversampling rate
  writeByte(BME280_ADDRESS, BME280_CTRL_HUM, 0x07 & Hosr);
  // Set T and P oversampling rates and sensor mode
  writeByte(BME280_ADDRESS, BME280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  writeByte(BME280_ADDRESS, BME280_CONFIG, SBy << 5 | IIRFilter << 2);
  // Read and store calibration data
  uint8_t calib[26];
  readBytes(BME280_ADDRESS, BME280_CALIB00, 26, &calib[0]);
  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
  dig_H1 = calib[25];
  readBytes(BME280_ADDRESS, BME280_CALIB26, 7, &calib[0]);
  dig_H2 = ( int16_t)((( int16_t) calib[1] << 8) | calib[0]);
  dig_H3 = calib[2];
  dig_H4 = ( int16_t)(((( int16_t) calib[3] << 8) | (0x0F & calib[4]) << 4) >> 4);
  dig_H5 = ( int16_t)(((( int16_t) calib[5] << 8) | (0xF0 & calib[4]) ) >> 4 );
  dig_H6 = calib[6];

  Serial.println("Calibration coefficients:");
  Serial.print("dig_T1 ="); 
  Serial.println(dig_T1);
  Serial.print("dig_T2 ="); 
  Serial.println(dig_T2);
  Serial.print("dig_T3 ="); 
  Serial.println(dig_T3);
  Serial.print("dig_P1 ="); 
  Serial.println(dig_P1);
  Serial.print("dig_P2 ="); 
  Serial.println(dig_P2);
  Serial.print("dig_P3 ="); 
  Serial.println(dig_P3);
  Serial.print("dig_P4 ="); 
  Serial.println(dig_P4);
  Serial.print("dig_P5 ="); 
  Serial.println(dig_P5);
  Serial.print("dig_P6 ="); 
  Serial.println(dig_P6);
  Serial.print("dig_P7 ="); 
  Serial.println(dig_P7);
  Serial.print("dig_P8 ="); 
  Serial.println(dig_P8);
  Serial.print("dig_P9 ="); 
  Serial.println(dig_P9);
  Serial.print("dig_H1 ="); 
  Serial.println(dig_H1);
  Serial.print("dig_H2 ="); 
  Serial.println(dig_H2);
  Serial.print("dig_H3 ="); 
  Serial.println(dig_H3);
  Serial.print("dig_H4 ="); 
  Serial.println(dig_H4);
  Serial.print("dig_H5 ="); 
  Serial.println(dig_H5);
  Serial.print("dig_H6 ="); 
  Serial.println(dig_H6);
  Serial.println(" ");
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t BME280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22integer and 10fractional bits).
// Output value of “47445”represents 47445/1024= 46.333%RH
uint32_t BME280_compensate_H(int32_t adc_H)
{
int32_t var;

var = (t_fine - ((int32_t)76800));
var = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var)) +
((int32_t)16384)) >> 15) * (((((((var * ((int32_t)dig_H6)) >> 10) * (((var *
((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
var = (var - (((((var >> 15) * (var >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
var = (var < 0 ? 0 : var); 
var = (var > 419430400 ? 419430400 : var);
return(uint32_t)(var >> 12);
}

void checkCCS811Status() 
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
  else { Serial.println("No error detected!");}
  
  Serial.println(" ");
  }

  void CCS811init()
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


    void CCS811Compensate()
  {
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

      digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor    
      Wire.beginTransmission(CCS811_ADDRESS);  // Initialize the Tx buffer
      for(int ii = 0; ii < 6; ii++) {
        Wire.write(temp[ii]);           // Put slave register address in Tx buffer
      }
      Wire.endTransmission();           // Send the Tx bufferdigitalWrite(CCS811Enable, HIGH); // set HIGH to disable the CCS811 air quality sensor
  }
 
// simple function to scan for I2C devices on the bus
void I2Cscan() 
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

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t readByte(uint8_t address, uint8_t subAddress) {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }
