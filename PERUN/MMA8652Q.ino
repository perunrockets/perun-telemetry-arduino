/* MMA8652Q Example Code
 Original sketch by: Jim Lindblom, Kris Winer modified Kamlo 2017
 SparkFun Electronics
 date: November 17, 2011
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.

 Define registers per MMA8652Q, Document Number: MMA8652FC
 Data Sheet: Technical Data Rev. 2.0, 02/2013 3-Axis, 12-bit/8-bit Digital Accelerometer
 Freescale Semiconductor Data Sheet
*/

#define MMA8652_STATUS           0x00
#define MMA8652_F_STATUS         0x00
#define MMA8652_OUT_X_MSB        0x01    
#define MMA8652_OUT_X_LSB        0x02
#define MMA8652_OUT_Y_MSB        0x03
#define MMA8652_OUT_Y_LSB        0x04
#define MMA8652_OUT_Z_MSB        0x05
#define MMA8652_OUT_Z_LSB        0x06
#define MMA8652_F_SETUP          0x09
#define MMA8652_TRIG_CFG         0x0A
#define MMA8652_SYSMOD           0x0B
#define MMA8652_INT_SOURCE       0x0C
#define MMA8652_WHO_AM_I         0x0D   
#define MMA8652_XYZ_DATA_CFG     0x0E
#define MMA8652_HP_FILTER_CUTOFF 0x0F
#define MMA8652_PL_STATUS        0x10
#define MMA8652_PL_CFG           0x11
#define MMA8652_PL_COUNT         0x12
#define MMA8652_PL_BF_ZCOMP      0x13
#define MMA8652_P_L_THS_REG      0x14
#define MMA8652_FF_MT_CFG        0x15
#define MMA8652_FF_MT_SRC        0x16
#define MMA8652_FF_MT_THS        0x17
#define MMA8652_FF_MT_COUNT      0x18
#define MMA8652_TRANSIENT_CFG    0x1D
#define MMA8652_TRANSIENT_SRC    0x1E
#define MMA8652_TRANSIENT_THS    0x1F
#define MMA8652_TRANSIENT_COUNT  0x20
#define MMA8652_PULSE_CFG        0x21
#define MMA8652_PULSE_SRC        0x22
#define MMA8652_PULSE_THSX       0x23
#define MMA8652_PULSE_THSY       0x24
#define MMA8652_PULSE_THSZ       0x25
#define MMA8652_PULSE_TMLT       0x26
#define MMA8652_PULSE_LTCY       0x27
#define MMA8652_PULSE_WIND       0x28
#define MMA8652_ASLP_COUNT       0x29
#define MMA8652_CTRL_REG1        0x2A
#define MMA8652_CTRL_REG2        0x2B
#define MMA8652_CTRL_REG3        0x2C
#define MMA8652_CTRL_REG4        0x2D
#define MMA8652_CTRL_REG5        0x2E
#define MMA8652_OFF_X            0x2F
#define MMA8652_OFF_Y            0x30
#define MMA8652_OFF_Z            0x31

#define MMA8652_ADDRESS 0x1D  // SA0 is high, 0x1C if low
//#define MMA8652_ADDRESS 0x1C


// Set initial input parameters
enum accelFSR {
  AFS_2g = 0,
  AFS_4g,
  AFS_8g
};

enum accelODR {
  AODR_800HZ = 0, // 200 Hz
  AODR_400HZ,
  AODR_200HZ,
  AODR_100HZ,
  AODR_50HZ,
  AODR_12_5HZ, // 12.5 Hz, etc.
  AODR_6_25HZ,
  AODR_1_56HZ
};

uint8_t accelFSR = AFS_2g; // Specify sensor full scale
uint8_t accelODR = AODR_200HZ;
float aRes; // scale resolutions per LSB for the sensor // Set the scale below either 2, 4 or 8
int16_t accelCount[3];  // Stores the 12-bit signed value
float ax, ay, az;       // Stores the real accel value in g's
boolean accel_sleepMode = false;


void getAres() {
  switch (accelFSR)
  {
 	      // Possible accelerometer scales (and their register bit settings) are:
	      // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2g:
          aRes = 2.0/2048.0;
          break;
    case AFS_4g:
          aRes = 4.0/2048.0;
          break;
    case AFS_8g:
          aRes = 8.0/2048.0;
          break;
  }
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MMA8652_ADDRESS, MMA8652_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 4;
  destination[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 4;
  destination[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 4;
}


void calibrateMMA8652()
{
  int32_t accel_bias[3] = {0, 0, 0};
  uint16_t ii, fcount;
  int16_t temp[3];
  
  // Clear all interrupts by reading the data output and F_STATUS registers
  readAccelData(temp);
  readByte(MMA8652_ADDRESS, MMA8652_F_STATUS);
  
  MMA8652Standby();  // Must be in standby to change registers

  writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1, 0x01);      // select 100 Hz ODR
  fcount = 250;                                     // sample for 1 second
  writeByte(MMA8652_ADDRESS, MMA8652_XYZ_DATA_CFG, 0x00);   // select 2 g full scale
  uint16_t accelsensitivity = 1024;                 // 1024 LSB/g

  MMA8652Active();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
  readBytes(MMA8652_ADDRESS, MMA8652_OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
  temp[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 4;
  temp[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 4;
  temp[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 4;
  
  accel_bias[0] += (int32_t) temp[0];
  accel_bias[1] += (int32_t) temp[1];
  accel_bias[2] += (int32_t) temp[2];
  
  delay(15);  // wait for a new data reading (100 Hz)
  }
 
  accel_bias[0] /= (int32_t) fcount; // get average values
  accel_bias[1] /= (int32_t) fcount;
  accel_bias[2] /= (int32_t) fcount;
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  rawData[0] = (-accel_bias[0]/2) & 0xFF; // get average values
  rawData[1] = (-accel_bias[1]/2) & 0xFF; // get average values
  rawData[2] = (-accel_bias[2]/2) & 0xFF; // get average values
  
  MMA8652Standby();  // Must be in standby to change registers
  
  writeByte(MMA8652_ADDRESS, MMA8652_OFF_X, rawData[0]); // X-axis compensation  
  writeByte(MMA8652_ADDRESS, MMA8652_OFF_Y, rawData[1]); // Y-axis compensation  
  writeByte(MMA8652_ADDRESS, MMA8652_OFF_Z, rawData[2]); // z-axis compensation 

  MMA8652Active();  // Set to active to start reading
}
  
  
// Set up sensor software reset
void MMA8652Reset() 
{
writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2, 0x40); // set reset bit to 1 to assert software reset to zero at end of boot process
}

// Allow user compensation of acceleration errors
void MMA8652Offsets()
{
   MMA8652Standby();  // Must be in standby to change registers
   
   // Factory settings are pretty good; the settings below produce 1 mg error or less at 2 g full scale! For the device at rest on my table 
   // these values partially compensate for the slope of the table and the slope of the sensor in my breadboard. It is a pretty stable setup!
   // For negative values use 2's complement, i.e., -2 mg = 0xFF, etc.
   writeByte(MMA8652_ADDRESS, MMA8652_OFF_X, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MMA8652_ADDRESS, MMA8652_OFF_Y, 0x00); // Y-axis compensation; this is 0 mg
   writeByte(MMA8652_ADDRESS, MMA8652_OFF_Z, 0x00); // z-axis compensation; this is  0 mg adjustment
   
   MMA8652Active();  // Set to active to start reading
}

// Initialize the MMA8652 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8652Q
// Feel free to modify any values, these are settings that work well for me.
void initMMA8652()
{
  MMA8652Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
    writeByte(MMA8652_ADDRESS, MMA8652_XYZ_DATA_CFG, accelFSR);  

   // First clear CTRL_REG1
    writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1, 0x00);
   // Setup the 3 data rate bits, from 0 to 7
    writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1) & ~(0x38));
    if (accelODR <= 7)
    writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1) | (accelODR << 3));      
    // set resolution
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2) & ~(0x03)); // clear bits 0 and 1
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2) |  (0x02)); // select normal(00) or high resolution (10) mode
    
// These settings have to do with setting up the sleep mode and should probably be broken up into a separate function
// set Auto-WAKE sample frequency when the device is in sleep mode

     writeByte(MMA8652_ADDRESS, 0x29, 0x40 ); // sleep after ~36 seconds of inactivity at 6.25 Hz ODR

     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1) & ~(0xC0)); // clear bits 7 and 8
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1) |  (0xC0)); // select 1.56 Hz sleep mode sample frequency for low power

  // set sleep power mode scheme
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2) & ~(0x18)); // clear bits 3 and 4
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2) |  (0x18)); // select low power mode
     
  // Enable auto SLEEP
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2) & ~(0x04)); // clear bit 2
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG2) |  (0x04)); // enable auto sleep mode

  // set sleep mode interrupt scheme
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG3, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG3) & ~(0x3C)); // clear bits 3, 4, 5, and 6
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG3, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG3) |  (0x3C)); // select wake on transient, orientation change, pulse, or freefall/motion detect
     
   // Enable Auto-SLEEP/WAKE interrupt
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG4, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG4) & ~(0x80)); // clear bit 7
     writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG4, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG4) |  (0x80)); // select  Auto-SLEEP/WAKE interrupt enable
   
  // Set up portrait/landscape registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  writeByte(MMA8652_ADDRESS, MMA8652_PL_CFG, 0x40);        // 1. Enable P/L
 // writeByte(MMA8652_ADDRESS, PL_BF_ZCOMP, 0x44); // 2. 29deg z-lock (don't think this register is actually writable)
 // writeByte(MMA8652_ADDRESS, P_L_THS_REG, 0x84); // 3. 45deg thresh, 14deg hyst (don't think this register is writable either)
  writeByte(MMA8652_ADDRESS, MMA8652_PL_COUNT, 0x50);      // 4. debounce counter at 100ms (at 800 hz)

  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  writeByte(MMA8652_ADDRESS, MMA8652_PULSE_CFG, 0x7F);  // 1. enable single/double taps on all axes
  // writeByte(MMA8652_ADDRESS, PULSE_CFS, 0x55);  // 1. single taps only on all axes
  // writeByte(MMA8652_ADDRESS, PULSE_CFS, 0x6A);  // 1. double taps only on all axes
  writeByte(MMA8652_ADDRESS, MMA8652_PULSE_THSX, 0x04);  // 2. x thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(MMA8652_ADDRESS, MMA8652_PULSE_THSY, 0x04);  // 2. y thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(MMA8652_ADDRESS, MMA8652_PULSE_THSZ, 0x04);  // 2. z thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(MMA8652_ADDRESS, MMA8652_PULSE_TMLT, 0x30);  // 3. 2.55s time limit at 100Hz odr, this is very dependent on data rate, see the app note
  writeByte(MMA8652_ADDRESS, MMA8652_PULSE_LTCY, 0xA0);  // 4. 5.1s 100Hz odr between taps min, this also depends on the data rate
  writeByte(MMA8652_ADDRESS, MMA8652_PULSE_WIND, 0xFF);  // 5. 10.2s (max value)  at 100 Hz between taps max

  // Set up motion detection
  writeByte(MMA8652_ADDRESS, MMA8652_FF_MT_CFG, 0x58); // Set motion flag on x and y axes
  writeByte(MMA8652_ADDRESS, MMA8652_FF_MT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set threshold to 0.25 g
  writeByte(MMA8652_ADDRESS, MMA8652_FF_MT_COUNT, 0x8); // Set debounce to 0.08 s at 100 Hz

  // Set up interrupt 1 and 2
  writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG3, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
  writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG3, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts
     
 // writeByte(MMA8652_ADDRESS, 0x2C, 0x02);  // Active high, push-pull interrupts

  writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG4, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
  writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG4, readByte(MMA8652_ADDRESS, MMA8652_CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled
   
  writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2

  MMA8652Active();  // Set to active to start reading
}

// Sets the MMA8652 to standby mode.
// It must be in standby to change most register settings
void MMA8652Standby()
{
  byte c = readByte(MMA8652_ADDRESS, 0x2A);
  writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1, c & ~(0x01));
}

// Sets the MMA8652 to active mode.
// Needs to be in this mode to output data
void MMA8652Active()
{
  byte c = readByte(MMA8652_ADDRESS, 0x2A);
  writeByte(MMA8652_ADDRESS, MMA8652_CTRL_REG1, c | 0x01);
}


void ReadMMA8652DataNormalise(float * destination) {
	if(readByte(MMA8652_ADDRESS, MMA8652_STATUS) & 0x08) {  // When this bit set, all axes have new data
  
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();                  // get accelerometer sensitivity
    //ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    //ay = (float)accelCount[1]*aRes;  // also subtract averaged accelerometer biases
    //az = (float)accelCount[2]*aRes;  

    destination[0] = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    destination[1] = (float)accelCount[1]*aRes;  // also subtract averaged accelerometer biases
    destination[2] = (float)accelCount[2]*aRes;  
	}
}

boolean StartMMA8652() {

  byte c = readByte(MMA8652_ADDRESS, MMA8652_WHO_AM_I);  // Read WHO_AM_I register

  if (c == 0x4A) { // WHO_AM_I should always be 0x4A
   
    MMA8652Reset(); // Start by resetting sensor device to default settings
    calibrateMMA8652();
    initMMA8652();  // init the accelerometer if communication is OK
    //Serial.println("MMA8652Q is online...");
	delay(1000);
	return true;
  }
  else
  {
    //Serial.print("Could not connect to MMA8652Q: 0x");
	return false;
  }
}

