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
 *  This sketch uses default SDA/SCL pins on the Ladybug development board.
 *  The BME280 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 *  mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 *  only 1 microAmp. The choice will depend on the application.
 
 Library may be used freely and without limit with attribution.
 
  */
#include "Wire.h"   
#include <RTC.h>
#include "BME280.h"
#include "CCS811.h"

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16 // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16 // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16 // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_16, Hosr = H_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_021ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate

float Temperature, Pressure, Humidity; // stores BME280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp, compTemp;   // pressure and temperature raw count output for BME280
uint32_t compHumidity, compPress;      // pressure and humidity compensated output for BME280
int16_t rawHumidity;                   // variables to hold raw BME280 humidity value

float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

uint32_t delt_t = 0, count = 0, sumCount = 0, slpcnt = 0;  // used to control display output rate

// RTC set up
/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 16;
const uint8_t hours = 9;

/* Change these values to set the current initial date */
const byte day = 10;
const byte month = 1;
const byte year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

float VDDA, VBAT;
const byte VbatMon = A4;

BME280 BME280; // instantiate BME280 class

// CCS811 definitions
#define CCS811_intPin  A1
#define CCS811_wakePin A2

/* Specify CCS811 sensor parameters
 *  Choices are   dt_idle , dt_1sec, dt_10sec, dt_60sec
 */
uint8_t AQRate = dt_10sec;  // set the sample rate
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // array to hold the raw data
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;

bool newCCS811Data  = false; // boolean flag for interrupt

CCS811 CCS811(CCS811_intPin, CCS811_wakePin); // instantiate CCS811 class

void setup()
{
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led on

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs

  Wire.begin(); // set master mode 
  Wire.setClock(100000); // I2C frequency at 400 kHz  
  delay(1000);
  
  //Enable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor

  BME280.I2Cscan(); // should detect BME280 at 0x77, BMA280 at 0x18, and CCS811 at 0x5A

  //Disable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor

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
  byte d = BME280.getChipID();  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the WHO_AM_I register of the CCS811 this is a good test of communication
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  byte e = CCS811.getChipID();
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor
  Serial.print("CCS811 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0x81, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(d == 0x60 && e == 0x81 ) {

   Serial.println("BME280+CCS811 are online..."); Serial.println(" ");

   BME280.resetBME280(); // reset BME280 before initilization
   delay(100);

   BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy); // Initialize BME280 altimeter

    // initialize CCS811 and check version and status
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.CCS811init(AQRate);
    digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor

    attachInterrupt(CCS811_intPin,  myinthandler, FALLING); // enable CCS811 interrupt
 
    }
    else 
    if(d != 0x60) Serial.println(" BME280 not functioning!");    
    if(e != 0x81) Serial.println(" CCS811 not functioning!");    
}

void loop()
{
    // CCS811 data
    Serial.println("CCS811:");
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.compensateCCS811(compHumidity, compTemp); // compensate CCS811 using BME280 humidity and temperature
    digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor

    // If intPin goes LOW, all data registers have new data
    if(newCCS811Data == true) {  // On interrupt, read data
    newCCS811Data = false;  // reset newData flag
     
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor

    CCS811.readCCS811Data(rawData);

    eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
    TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
    Current = (rawData[6] & 0xFC) >> 2;
    Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f), 3; 
    }
            
    digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor 

    Serial.print("Eq CO2 in ppm = "); Serial.println(eCO2);
    Serial.print("TVOC in ppb = "); Serial.println(TVOC);
    Serial.print("Sensor current (uA) = "); Serial.println(Current);
    Serial.print("Sensor voltage (V) = "); Serial.println(Voltage, 2);  
    Serial.println(" ");
    
    // Serial print and/or display at 2Hz rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update serial once per half-second independent of read rate
   
    rawTemp =  BME280.readBME280Temperature();
    compTemp = BME280.BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readBME280Pressure();
    pressure = (float) BME280.BME280_compensate_P(rawPress)/25600.f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    rawHumidity =  BME280.readBME280Humidity();
    compHumidity = BME280.BME280_compensate_H(rawHumidity);
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
      
    digitalWrite(myLed, !digitalRead(myLed)); // blink led at end of loop
    count = millis();  
    }  
    
      delay(900);
      
 //     STM32.sleep();    // time out in stop mode to save power
}

//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler()
{
  newCCS811Data = true;
}
