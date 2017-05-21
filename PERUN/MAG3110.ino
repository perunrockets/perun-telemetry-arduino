 /* MAG3110 Example Code
 Original sketch by: Kris Winer, May 27, 2014 modified Kamlo 2017
 
 Includes reset, magnetometer initialization and calibration, as well as parameterizing the register addresses. 

 This code should provide example usage for most features of
 the MAG3110 3-axis, I2C 16-bit magnetometer. In the loop function
 the magnetoometer interrupt outputs will be polled, and either
 the x/y/z mag data will be output, or magnetic threshold detection will be displayed.

 Note: The MAG3110 is an I2C sensor; here we make use of the Arduino Wire library.Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 
 Define registers per MAG3110, Document Number: MAG3110FC
 Data Sheet: Technical Data Rev. 2.0, 02/2013 3-Axis, 12-bit/8-bit Digital Accelerometer
 Freescale Semiconductor Data Sheet
 */
 
#define MAG3110_DR_STATUS        0x00
#define MAG3110_OUT_X_MSB        0x01    
#define MAG3110_OUT_X_LSB        0x02
#define MAG3110_OUT_Y_MSB        0x03
#define MAG3110_OUT_Y_LSB        0x04
#define MAG3110_OUT_Z_MSB        0x05
#define MAG3110_OUT_Z_LSB        0x06
#define MAG3110_WHO_AM_I         0x07   
#define MAG3110_SYSMOD           0x08
#define MAG3110_OFF_X_MSB        0x09    
#define MAG3110_OFF_X_LSB        0x0A
#define MAG3110_OFF_Y_MSB        0x0B
#define MAG3110_OFF_Y_LSB        0x0C
#define MAG3110_OFF_Z_MSB        0x0D
#define MAG3110_OFF_Z_LSB        0x0E
#define MAG3110_DIE_TEMP         0x0F
#define MAG3110_CTRL_REG1        0x10
#define MAG3110_CTRL_REG2        0x11

#define MAG3110_ADDRESS 0x0E  // SA0 is high, 0xOE if low
//#define MAG3110_ADDRESS 0x0E


// Set initial input parameters
enum magODR {
  mODR_80Hz_16os = 0,  // 80 Hz data output rate with 16 times oversampling (1280 Hz is max ADC rate)
  mODR_40Hz_32os,
  mODR_20Hz_64os,
  mODR_10Hz_128os,
  mODR_40Hz_16os,  
  mODR_20Hz_32os,
  mODR_10Hz_64os,
  mODR_5Hz_128os,
  mODR_20Hz_16os,  
  mODR_10Hz_32os,
  mODR_5Hz_64os,
  mODR_2_5Hz_128os,
  mODR_10Hz_16os,  
  mODR_5Hz_32os,
  mODR_2_5Hz_64os,
  mODR_1_25Hz_128os,
  mODR_5Hz_16os,  
  mODR_2_5Hz_32os,
  mODR_1_25Hz_64os,
  mODR_0_63Hz_128os,
  mODR_2_5Hz_16os,  
  mODR_1_25Hz_32os,
  mODR_0_63Hz_64os,
  mODR_0_31Hz_128os,
  mODR_1_25Hz_16os,  
  mODR_0_63Hz_32os,
  mODR_0_31Hz_64os,
  mODR_0_16Hz_128os,
  mODR_0_63Hz_16os,  
  mODR_0_31Hz_32os,
  mODR_0_11Hz_64os,
  mODR_0_08Hz_128os
};


// Specify sensor sample data rate and oversampling
uint8_t magODR = mODR_80Hz_16os;
int16_t magCount[3];  // Stores the 12-bit signed value
float mx, my, mz, mtemperature;       // Stores the real accel value in g's


void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MAG3110_ADDRESS, MAG3110_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t) rawData[0] << 8 | rawData[1]);
  destination[1] = ((int16_t) rawData[2] << 8 | rawData[3]);
  destination[2] = ((int16_t) rawData[4] << 8 | rawData[5]);
}

int8_t MAG3110readTempData()
{
  return (int8_t) readByte(MAG3110_ADDRESS, MAG3110_DIE_TEMP);  // Read the 8-bit 2's complement data register 
}
  
  
// Set up sensor software reset
void MAG3110Reset() 
{
writeByte(MAG3110_ADDRESS, MAG3110_CTRL_REG2, 0x10); // set reset bit to 1 to assert software reset to zero at end of boot process
}

// Allow user compensation of acceleration errors
void MAG3110Offsets()
{
   MAG3110Standby();  // Must be in standby to change registers
   
   writeByte(MAG3110_ADDRESS, MAG3110_OFF_X_MSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, MAG3110_OFF_X_LSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, MAG3110_OFF_Y_MSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, MAG3110_OFF_Y_LSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, MAG3110_OFF_Z_MSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, MAG3110_OFF_Z_LSB, 0x00); // X-axis compensation; this is 0 mg
   
   MAG3110Active();  // Set to active to start reading
}

// Initialize the MAG3110 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MAG3110
// Feel free to modify any values, these are settings that work well for me.
void initMAG3110()
{
  MAG3110Standby();  // Must be in standby to change registers

  // Set up the magnetometer sample rate and oversample ratio
    writeByte(MAG3110_ADDRESS, MAG3110_CTRL_REG1, magODR << 3);  
  // Enable automatic magnetic sensor resets
    writeByte(MAG3110_ADDRESS, MAG3110_CTRL_REG2, 0x80);  // set normal mode, correct with user offset registers

  MAG3110Active();  // Set to active to start reading
}

// Sets the MAG3110 to standby mode.
// It must be in standby to change most register settings
void MAG3110Standby()
{
  byte c = readByte(MAG3110_ADDRESS, MAG3110_CTRL_REG1);
  writeByte(MAG3110_ADDRESS, MAG3110_CTRL_REG1, c & ~(0x01));
}

// Sets the MAG3110 to active mode.
// Needs to be in this mode to output data
void MAG3110Active()
{
  byte c = readByte(MAG3110_ADDRESS, MAG3110_CTRL_REG1);
  writeByte(MAG3110_ADDRESS, MAG3110_CTRL_REG1, c | 0x01);  
}

void ReadMAG3110DataNormalise(float * destination) {
  // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
  if(readByte(MAG3110_ADDRESS, MAG3110_DR_STATUS) & 0x08)  { // When this bit set, all axes have new data
 
    readMagData(magCount);               // Read the x/y/z adc values
    destination[0] = (float)magCount[0]*10./32768.;  // get actual Gauss value 
    destination[1] = (float)magCount[1]*10./32768.;   
    destination[2] = (float)magCount[2]*10./32768.;  
    
    int8_t tempCount = MAG3110readTempData();  // Read the x/y/z adc values
    destination[3] = (float) tempCount + 26.; // Temperature in degrees Centigrade
  }   
}

boolean StartMAG3110() {

  byte c = readByte(MAG3110_ADDRESS, MAG3110_WHO_AM_I);  // Read WHO_AM_I register

  if (c == 0xC4) // WHO_AM_I should always be 0x4A
  {  
    //MAG3110Reset();  // Start by resetting sensor device to default settings
    //MAG3110Offsets(); // Apply user offsets
    initMAG3110();   // init the accelerometer if communication is OK
    //Serial.println("MAG3110 is online...");
	delay(1000);
	return true;
  }
  else
  {
    //Serial.print("Could not connect to MAG3110: 0x");
	return false;
  }
}


// CALIBRATE //


void MAG3110StartCalibration() {

	writeByte(MAG3110_ADDRESS, MAG3110_CTRL_REG2, 0x80);  // Enable automatic resets
	readByte(MAG3110_ADDRESS, MAG3110_WHO_AM_I); // Read WHO_AM_I Register  
	writeByte(MAG3110_ADDRESS, MAG3110_CTRL_REG1, 0x11); // ODR 20Hz (0.05s), Active mode
	
	  int16_t Xout_16_bit, Yout_16_bit, Zout_16_bit; 
	  int16_t Xout_16_bit_avg, Yout_16_bit_avg, Zout_16_bit_avg;  
    int16_t Xout_16_bit_max, Yout_16_bit_max, Zout_16_bit_max;  
    int16_t Xout_16_bit_min, Yout_16_bit_min, Zout_16_bit_min;  
    int16_t i = 0;  
     
	Serial.println("Start calibration MAG3110, wait ~10s");	
		
    while (i < 200) // Calibration process ~10s (200 samples * 1/20Hz)  
    {  
        if (readByte(MAG3110_ADDRESS, MAG3110_DR_STATUS) & 0x08)  {   
      
				uint8_t rawData[6];  // x/y/z accel register data stored here
				readBytes(MAG3110_ADDRESS, MAG3110_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
				Xout_16_bit = ((int16_t) rawData[0] << 8 | rawData[1]);
				Yout_16_bit = ((int16_t) rawData[2] << 8 | rawData[3]);
				Zout_16_bit = ((int16_t) rawData[4] << 8 | rawData[5]);
	  
                if (i == 0)  
                {  
                    Xout_16_bit_max = Xout_16_bit;  
                    Xout_16_bit_min = Xout_16_bit;  
      
                    Yout_16_bit_max = Yout_16_bit;  
                    Yout_16_bit_min = Yout_16_bit;  
      
                    Zout_16_bit_max = Zout_16_bit;  
                    Zout_16_bit_min = Zout_16_bit;  
                }  
      
                // Check to see if current sample is the maximum or minimum X-axis value  
                if (Xout_16_bit > Xout_16_bit_max) {Xout_16_bit_max = Xout_16_bit;}  
                if (Xout_16_bit < Xout_16_bit_min) {Xout_16_bit_min = Xout_16_bit;}  
      
                // Check to see if current sample is the maximum or minimum X-axis value  
                if (Yout_16_bit > Yout_16_bit_max) {Yout_16_bit_max = Yout_16_bit;}  
                if (Yout_16_bit < Yout_16_bit_min) {Yout_16_bit_min = Yout_16_bit;}  
      
                // Check to see if current sample is the maximum or minimum X-axis value  
                if (Zout_16_bit > Zout_16_bit_max) {Zout_16_bit_max = Zout_16_bit;}  
                if (Zout_16_bit < Zout_16_bit_min) {Zout_16_bit_min = Zout_16_bit;}  
      
                i++;  
            }  
			Serial.println("Calibrating...");	
    }  
      
      
    Xout_16_bit_avg = (Xout_16_bit_max + Xout_16_bit_min) / 2;    // X-axis hard-iron offset  
    Yout_16_bit_avg = (Yout_16_bit_max + Yout_16_bit_min) / 2;    // Y-axis hard-iron offset  
    Zout_16_bit_avg = (Zout_16_bit_max + Zout_16_bit_min) / 2;    // Z-axis hard-iron offset  
      
    // Left-shift by one as magnetometer offset registers are 15-bit only, left justified  
    Xout_16_bit_avg <<= 1;  
    Yout_16_bit_avg <<= 1;  
    Zout_16_bit_avg <<= 1;  
      
    MAG3110Standby();
      
    writeByte(MAG3110_ADDRESS, MAG3110_OFF_X_MSB, (char)((Xout_16_bit_avg >>8) & 0xFF)); 
    writeByte(MAG3110_ADDRESS, MAG3110_OFF_X_LSB, (char)(Xout_16_bit_avg & 0xFF)); 
    writeByte(MAG3110_ADDRESS, MAG3110_OFF_Y_MSB, (char)((Yout_16_bit_avg >>8) & 0xFF));
    writeByte(MAG3110_ADDRESS, MAG3110_OFF_Y_LSB, (char)(Yout_16_bit_avg & 0xFF));
    writeByte(MAG3110_ADDRESS, MAG3110_OFF_Z_MSB, (char)((Zout_16_bit_avg >>8) & 0xFF));
    writeByte(MAG3110_ADDRESS, MAG3110_OFF_Z_LSB, (char)(Zout_16_bit_avg & 0xFF));  
	   
	MAG3110Active();
	
	/*
	uint8_t rawDataOff[6];
	int16_t Xoff, Yoff, Zoff; 
	readBytes(MAG3110_ADDRESS, MAG3110_OFF_X_MSB, 6, &rawDataOff[0]);
	Xoff = ((int16_t) rawDataOff[0] << 8 | rawDataOff[1]);
	Yoff = ((int16_t) rawDataOff[2] << 8 | rawDataOff[3]);
	Zoff = ((int16_t) rawDataOff[4] << 8 | rawDataOff[5]);
	
	Serial.print("x-offset = "); Serial.print(Xoff);
	Serial.print(" y-offset = "); Serial.print(Yoff);
	Serial.print(" z-offset = "); Serial.println(Zoff);
	*/
	
	Serial.println("Finish calibration MAG3110");
	Serial.print("x-offset = "); Serial.print(Xout_16_bit_avg);
	Serial.print(" y-offset = "); Serial.print(Yout_16_bit_avg);
	Serial.print(" z-offset = "); Serial.println(Zout_16_bit_avg);	

}

