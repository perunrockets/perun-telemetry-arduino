/* PERUN ROCKETS TELEMETRY v0.2 
 * Original sketch by Kamlo 2017
*/

#include <Wire.h>
#include "i2c.h"
#include <SD.h>
#include <MCP342x.h>
#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>
#include <Kalman.h>

// ACC,GYRO,MAG, MADWICK FILTER AHRS
float ax_,ay_,az_,gx_,gy_,gz_,mx_,my_,mz_;
float temp_m,temp_g;
float rollMadgwick, pitchMadgwick, yawMadgwick;
float rollRaw, pitchRaw, yawRaw;

Madgwick filterMadgwick;
//Mahony filterMadgwick;
unsigned long microsNow = 0;
unsigned long microsPerReading, microsPrevious;

// MCP342X
MCP342x adc = MCP342x(0x68);
MCP342x::Config configCh1(MCP342x::channel1, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1);
MCP342x::Config configCh2(MCP342x::channel2, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1);
MCP342x::Config statusAdc;
int chanelConversion = 1;
int readyConversion = 1;
int16_t valueRawAdcCh1,valueRawAdcCh2 = 0;
float MPX2010valueKpa = 0;
float MPXHZ6115valueKpa = 0;
long valueAdc = 0;
uint8_t errAdc;

float AltitudeFromPressure = 0;
float WindSpeedFromPressure = 0;

//KALMAN FILTER PRESURE
double PresureMPX2010Kalman, PresureMPXHZ6115Kalman;
Kalman presureMPX2010Filter(0.125,32,1023,0);
Kalman presureMPXHZ6115Filter(0.125,32,1023,0);

unsigned long microsRuning = 0;
unsigned long microsRuningTmp= 0;
unsigned long microsRuningOld = 0;

// TEMP
int STAT_STARTUP, STAT_LIFTOF, STAT_DEPLOY_P, STAT_FINISH;


void setup() {
  Serial.begin(500000);
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  
  StartFXAS2100();
  StartMMA8652();
  StartMAG3110();

  MCP342x::generalCallReset();
  
  InitSD();
  logHeader();

  //filterMadgwick.begin(25); 
  //microsPerReading = 1000000 / 25;

  filterMadgwick.begin(40);    //40
  microsPerReading = 1000000 / 50;   //50
  
  microsPrevious = micros();
  
}

void loop() {

  // SECOND COUNTER
  microsRuning = micros();
  
  float accelData[4];
  float gyroData[3];
  float magData[4];

  ReadMMA8652DataNormalise(accelData);
  ReadFXAS2100DataNormalise(gyroData);
  ReadMAG3110DataNormalise(magData);

  ax_ = accelData[0];
  ay_ = accelData[1];
  az_ = accelData[2];
  gx_ = gyroData[0];
  gy_ = gyroData[1];
  gz_ = gyroData[2];
  mx_ = magData[0];
  my_ = magData[1];
  mz_ = magData[2];
  
  temp_m = magData[3];
  temp_g = gyroData[3];

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

  //filterMadgwick.update(gx_, gy_, gz_, ax_, ay_, az_, my_, mx_, mz_);
  //filterMadgwick.update(gx_, gy_, -gz_, ax_, ay_, -az_, my_, mx_, mz_);
  filterMadgwick.updateIMU(gx_, gy_, gz_, ax_, ay_, az_);
  
  rollMadgwick = filterMadgwick.getRoll();
  pitchMadgwick = filterMadgwick.getPitch();
  yawMadgwick = filterMadgwick.getYaw();


  // VISUALISER
  Serial.print("Telemetry: ");
  Serial.print(yawMadgwick*(float)-1);
  Serial.print(" ");
  Serial.print(pitchMadgwick*(float)-1);
  Serial.print(" ");
  Serial.print(rollMadgwick);
  Serial.print(" ");
  Serial.print(AltitudeFromPressure);
  Serial.print(" ");
  Serial.print(WindSpeedFromPressure);
  Serial.print(" ");
  Serial.print((float)temp_g);
  Serial.print(" ");
  Serial.print((float)microsRuning/(float)1000000,0);
  Serial.print(" ");
  Serial.print(STAT_STARTUP);
  Serial.print(" ");
  Serial.print(STAT_LIFTOF);
  Serial.print(" ");
  Serial.print(STAT_DEPLOY_P);
  Serial.print(" ");
  Serial.println(STAT_FINISH);

  microsPrevious = microsPrevious + microsPerReading;
  }

 // AHRS DATA RAW
  pitchRaw = -(atan2(ax_, sqrt(ay_*ay_ + az_*az_))*180.0)/M_PI;
  rollRaw  = (atan2(ax_, az_)*180.0)/M_PI;
  yawRaw = atan2(my_, mx_);
  if(yawRaw < 0) yawRaw += 2*M_PI;
  if(yawRaw > 2*M_PI) yawRaw -= 2*M_PI;
  yawRaw = yawRaw * 180/ M_PI;

// MULTIPLEXING READ ADC
  long valueAdc = 0;
  uint8_t errAdc;
  if (readyConversion == 1) {
        switch (chanelConversion) {
            case 1:
              errAdc = adc.convert(configCh1);
            break;
            case 2:
              errAdc = adc.convert(configCh2); 
            break;
        }
      readyConversion = 0;
    }
    
    errAdc = adc.read(valueAdc, statusAdc);
    if (!errAdc && statusAdc.isReady()) { 
  
        switch (chanelConversion) {
            case 1:
                valueRawAdcCh1 = valueAdc;
                chanelConversion = 2;
              break;
              case 2:
                valueRawAdcCh2 = valueAdc;
                chanelConversion = 1;
              break;
         }
      readyConversion = 1;
    }

  // CALCULATE MPX2010, MPXHZ6115 TO kPa
  MPX2010valueKpa = convertMPX2010ToKpa(valueRawAdcCh1);
  MPXHZ6115valueKpa = convertMPXHZ6115ToKpa(valueRawAdcCh2);

  //KALMAN FILTER PRESURE
  PresureMPX2010Kalman = presureMPX2010Filter.getFilteredValue(MPX2010valueKpa);
  PresureMPXHZ6115Kalman = presureMPXHZ6115Filter.getFilteredValue(MPXHZ6115valueKpa);
  
  AltitudeFromPressure = getAltitudeFromPressure(101200, PresureMPXHZ6115Kalman); // 101200 test
  WindSpeedFromPressure = getWindSpeedFromPressure(PresureMPX2010Kalman);

/*
      // SIMULATION VISUALISER
      AltitudeFromPressure = 0;
      WindSpeedFromPressure = 0;
    
      if(((float)microsRuning/(float)1000000) > 15) {
      STAT_STARTUP = 1;
      }
      if(((float)microsRuning/(float)1000000) > 25) {
      STAT_LIFTOF = 1; 
      }
      if(((float)microsRuning/(float)1000000) > 35) {
      STAT_DEPLOY_P = 1;
      }
      if(((float)microsRuning/(float)1000000) > 45) {
      STAT_FINISH = 1;
      }
*/

    // 1 SECOND SD WRITE
      microsRuningTmp = (int)(microsRuning/(float)1000000);
      if(microsRuningTmp != microsRuningOld) {
      // LOG SD
      logTelemetry(microsRuning,ax_,ay_,az_,gx_,gy_,gz_,mx_,my_,mz_,temp_g,temp_m,pitchRaw,rollRaw,yawRaw,rollMadgwick,pitchMadgwick,yawMadgwick,valueRawAdcCh1,valueRawAdcCh2, MPX2010valueKpa,MPXHZ6115valueKpa,PresureMPX2010Kalman, PresureMPXHZ6115Kalman, AltitudeFromPressure, WindSpeedFromPressure);
      microsRuningOld = microsRuningTmp;
    }


}
