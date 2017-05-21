/* FXAS21000 
 Original sketch by: Kris Winer modified Kamlo 2017
 May 30, 2014
 
 FXAS21000 is a small, low-power, 3-axis yaw, pitch, and roll
 angular rate gyroscope. The full-scale range is adjustable from
 ±200°/s to ±1600°/s. It features both I2C and SPI interfaces. We parameterize the registers, 
 calibrate the gyro, get properly scaled angular rates.
 
 Define registers per Freescale Semiconductor, Inc.
 FXAS21000 Data Sheet: Advance Information Rev 1.1, 10/2013 3-Axis, 14-bit Digital MEMS Gyroscope
 Freescale Semiconductor Data Sheet
*/

#define FXAS21000_STATUS           0x00
#define FXAS21000_OUT_X_MSB        0x01    
#define FXAS21000_OUT_X_LSB        0x02
#define FXAS21000_OUT_Y_MSB        0x03
#define FXAS21000_OUT_Y_LSB        0x04
#define FXAS21000_OUT_Z_MSB        0x05
#define FXAS21000_OUT_Z_LSB        0x06
#define FXAS21000_DR_STATUS        0x07
#define FXAS21000_F_STATUS         0x08
#define FXAS21000_F_EVENT          0x0A
#define FXAS21000_INT_SRC_FLAG     0x0B
#define FXAS21000_WHO_AM_I         0x0C   // Should return 0xD1
#define FXAS21000_CTRL_REG0        0x0D
#define FXAS21000_RT_CFG           0x0E   
#define FXAS21000_RT_SRC           0x0F
#define FXAS21000_RT_THS           0x10
#define FXAS21000_RT_COUNT         0x11
#define FXAS21000_TEMP             0x12
#define FXAS21000_CTRL_REG1        0x13
#define FXAS21000_CTRL_REG2        0x14

#define FXAS21000_ADDRESS 0x20  // SA0 is high, 0x1C if low
//#define FXAS21000_ADDRESS 0x20


// Set initial input parameters
enum gyroFSR {
  GFS_1600DPS = 0,
  GFS_800DPS,
  GFS_400DPS,
  GFS_200DPS
};

enum gyroODR {
  GODR_200HZ = 0, // 200 Hz
  GODR_100HZ,
  GODR_50HZ,
  GODR_25HZ,
  GODR_12_5HZ,
  GODR_6_25HZ, // 6.25 Hz, etc.
  GODR_3_125HZ,
  GODR_1_5625HZ
};


uint8_t gyroFSR = GFS_200DPS; // Specify sensor full scale
uint8_t gyroODR = GODR_200HZ;
float gRes, gBias[3] = {0, 0, 0}; // scale resolutions per LSB for the sensors
int16_t gyroCount[3];  // Stores the 12-bit signed value
float gx, gy, gz, gtemperature;;  // Stores the real accel value in g's


///////////////////////////////////////////////////////////////////////////////
// Useful functions to access the FXAS21000 gyroscope
///////////////////////////////////////////////////////////////////////////////

void getGres() {
  switch (gyroFSR)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 200 DPS (11), 400 DPS (10), 800 DPS (01), and 1600 DPS  (00). 
    case GFS_1600DPS:
          gRes = 1600.0/8192.0;
          break;
    case GFS_800DPS:
          gRes = 800.0/8192.0;
          break;
    case GFS_400DPS:
          gRes = 400.0/8192.0;
          break;           
    case GFS_200DPS:
          gRes = 200.0/8192.0;
  }
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXAS21000_ADDRESS, FXAS21000_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2; // signed 14-bit integers
  destination[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
  destination[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
}

int8_t FXAS21000readTempData()
{
  return (int8_t) readByte(FXAS21000_ADDRESS, FXAS21000_TEMP);  // Read the 8-bit 2's complement data register 
}

 
void calibrateFXAS21000(float * gBias)
{
  int32_t gyro_bias[3] = {0, 0, 0};
  uint16_t ii, fcount;
  int16_t temp[3];
  
  // Clear all interrupts by reading the data output and STATUS registers
  readGyroData(temp);
  readByte(FXAS21000_ADDRESS, FXAS21000_STATUS);
  
  FXAS21000Standby();  // Must be in standby to change registers

  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, 0x08);   // select 50 Hz ODR
  fcount = 250;                                     // sample for 1 second
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG0, 0x03);   // select 200 deg/s full scale
  uint16_t gyrosensitivity = 41;                   // 40.96 LSB/deg/s

  FXAS21000Active();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
  readBytes(FXAS21000_ADDRESS, FXAS21000_OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
  temp[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2;
  temp[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
  temp[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
  
  gyro_bias[0] += (int32_t) temp[0];
  gyro_bias[1] += (int32_t) temp[1];
  gyro_bias[2] += (int32_t) temp[2];
  
  delay(25); // wait for next data sample at 50 Hz rate
  }
 
  gyro_bias[0] /= (int32_t) fcount; // get average values
  gyro_bias[1] /= (int32_t) fcount;
  gyro_bias[2] /= (int32_t) fcount;
  
  gBias[0] = (float)gyro_bias[0]/(float) gyrosensitivity; // get average values
  gBias[1] = (float)gyro_bias[1]/(float) gyrosensitivity; // get average values
  gBias[2] = (float)gyro_bias[2]/(float) gyrosensitivity; // get average values

  FXAS21000Ready();  // Set to ready
}
  
  
// Set up sensor software reset
void FXAS21000Reset() 
{
writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, 0x20); // set reset bit to 1 to assert software reset to zero at end of boot process
delay(100);
while(!(readByte(FXAS21000_ADDRESS, FXAS21000_INT_SRC_FLAG) & 0x08))  { // wait for boot end flag to be set
}
}


// Initialize the FXAS21000 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FXAS21000Q
// Feel free to modify any values, these are settings that work well for me.
void initFXAS21000()
{
  FXAS21000Standby();  // Must be in standby to change registers

  // Set up the full scale range to 200, 400, 800, or 1600 deg/s.
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG0, gyroFSR);      // write FSR

  // Setup the 3 data rate bits, 4:2
  if(gyroODR < 8) { // data rate can only be 0 to 7
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, gyroODR << 2); 
  }

  // Disable FIFO, route FIFO and rate threshold interrupts to INT2, enable data ready interrupt, route to INT1
  // Active HIGH, push-pull output driver on interrupts
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG2,  0x0E);
  
  // Set up rate threshold detection; at max rate threshold = FSR; rate threshold = THS*FSR/128
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_CFG, 0x07);         // enable rate threshold detection on all axes
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_THS, 0x00 | 0x0D);  // unsigned 7-bit THS, set to one-tenth FSR; set clearing debounce counter
  writeByte(FXAS21000_ADDRESS, FXAS21000_RT_COUNT, 0x04);       // set to 4 (can set up to 255)
        
  FXAS21000Active();  // Set to active to start reading
}

// Sets the FXAS21000 to standby mode.
// It must be in standby to change most register settings
void FXAS21000Standby()
{
  byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void FXAS21000Ready()
{
  byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
  writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c |   0x01);   // Set bit 0 to 1, ready mode; no data acquisition yet
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void FXAS21000Active()
{
 byte c = readByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1);
 writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
 writeByte(FXAS21000_ADDRESS, FXAS21000_CTRL_REG1, c |   0x02);   // Set bit 1 to 1, active mode; data acquisition enabled
}

void ReadFXAS2100DataNormalise(float * destination) {
	
  // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
  if(readByte(FXAS21000_ADDRESS, FXAS21000_DR_STATUS) & 0x08) {  // When this bit set, all axes have new data
  
	readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
    
    // Calculate the gyro value into actual degrees per second
    destination[0] = (float)gyroCount[0]*gRes - gBias[0];  // get actual gyro value, this depends on scale being set
    destination[1] = (float)gyroCount[1]*gRes - gBias[1];  
    destination[2] = (float)gyroCount[2]*gRes - gBias[2];   
	
	  int8_t tempCount = FXAS21000readTempData();  // Read the x/y/z adc values
    destination[3] = (float) tempCount; // Temperature in degrees Centigrade
  }
}

boolean StartFXAS2100() {
	
  //byte c = readByte(FXAS21000_ADDRESS, FXAS21000_WHO_AM_I);  // Read WHO_AM_I register
  
  //if (c == 0xD1) // WHO_AM_I should always be 0x2A
  //{  
    FXAS21000Reset(); // Start by resetting sensor device to default settings
    calibrateFXAS21000(gBias);
    initFXAS21000();  // init the accelerometer if communication is OK
	  delay(1000);
	//return true;
	//Serial.println("FXAS21000Q is online...");
  //}
  //else
  //{
    //Serial.println("Could not connect to FXAS21000Q: 0x");
	//return false;
  //}
}


