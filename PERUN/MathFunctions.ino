/* PERUN ROCKETS TELEMETRY v0.2 
 * Original sketch by Kamlo 2017
*/

float convertMPX2010ToKpa(float AdcRaw) {
  //float g = AdcRaw * 0.0000625;  //Sensitivity MCP3424 0.0000625 mV  // 16-bit, 1X Gain

  if(AdcRaw>32768-1) {
  AdcRaw=AdcRaw-65536-1;
  }
  float g = (AdcRaw*62.5)/1000;    // to milivolts 
  //AdcRaw = AdcRaw*RES/_PGA;

  g = g / 125.0; //Sensitivity MPX2010 2.5 mV/kPa = (125mV afer gain x50)
  return g;
}

float convertMPXHZ6115ToKpa(float AdcRaw) {
  //float g = AdcRaw * 0.0000625;  //Sensitivity MCP3424 0.0000625 mV  // 16-bit, 1X Gain
  
  if(AdcRaw>32768-1) {
  AdcRaw=AdcRaw-65536-1;
  }
  float g = (AdcRaw*62.5)/1000; //AdcRaw = AdcRaw*RES/_PGA; and conversion to milivolts 
  //g = g/2; // Po zastosowaniu dzielnika napięcia
  
  float kpa = ((g/2048)+0.095)/0.009; // 2048mV max MCP3424 in +chanel
 
  return kpa;
}

float getAltitudeFromPressure(long sealevelPressure, float Kpa) {
    //sealevelPressure = 101325;
    float pascalsPressure = Kpa*1000;
    float altitude = 44330 * (1.0 - pow(pascalsPressure / sealevelPressure, 0.1903));
    return altitude;
}

float getWindSpeedFromPressure(float Kpa) {
    //float windspeed = sqrt ( (2.0 * (Kpa-0.14) * 1000)/1.2);
    float windspeed = sqrt ( 2.0 * Kpa / 1.2 );
    //float windspeed = 2*sqrt ( (2.0 * (Kpa-0.14)*2000)/1.2);
    return windspeed;
}
