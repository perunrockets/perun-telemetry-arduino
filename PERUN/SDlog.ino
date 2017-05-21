/* PERUN ROCKETS TELEMETRY v0.2 
 * Original sketch by Kamlo 2017
*/

File LogFileSD;

boolean InitSD() {
  if (SD.begin(48)) {
    return true;
  } else {
    return false;
  }
}

void logHeader() {  
  LogFileSD = SD.open("log.csv", FILE_WRITE);

  String nag = "microsRuning;ax;ay;az;gx;gy;gz;mx;my;mz;gtemp;mtemp;pitchRaw;rollRaw;yawRaw;\ 1q54rollMadgwick;pitchMadgwick;yawMadgwick;valueRawAdcCh1;valueRawAdcCh2;MPX2010valueKpa;MPXHZ6115valueKpa;MPX2010valueKpaKalman;MPXHZ6115valueKpaKalman;AltitudeFromPressure;WindSpeedFromPressure;";
  LogFileSD.println(nag);
  LogFileSD.close();
}

void logTelemetry(float microsRuning, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz,float gtemp, float mtemp, float pitchRaw, float rollRaw, float yawRaw, float rollMadgwick, float pitchMadgwick, float yawMadgwick, float valueRawAdcCh1, float valueRawAdcCh2, float MPX2010valueKpa, float MPXHZ6115valueKpa,float MPX2010valueKpaKalman, float MPXHZ6115valueKpaKalman, float AltitudeFromPressure, float WindSpeedFromPressure) {  
LogFileSD = SD.open("log.csv", FILE_WRITE);
if (LogFileSD) {

LogFileSD.print((float)((float)microsRuning/1000000),2);
LogFileSD.print(";");

LogFileSD.print(ax);
LogFileSD.print(";");
LogFileSD.print(ay);
LogFileSD.print(";");
LogFileSD.print(az);
LogFileSD.print(";");

LogFileSD.print(gx);
LogFileSD.print(";");
LogFileSD.print(gy);
LogFileSD.print(";");
LogFileSD.print(gz);
LogFileSD.print(";");

LogFileSD.print(mx);
LogFileSD.print(";");
LogFileSD.print(my);
LogFileSD.print(";");
LogFileSD.print(mz);
LogFileSD.print(";");

LogFileSD.print(gtemp);
LogFileSD.print(";");
LogFileSD.print(mtemp);
LogFileSD.print(";");

LogFileSD.print(pitchRaw);
LogFileSD.print(";");
LogFileSD.print(rollRaw);
LogFileSD.print(";");
LogFileSD.print(yawRaw);
LogFileSD.print(";");

LogFileSD.print(rollMadgwick);
LogFileSD.print(";");
LogFileSD.print(pitchMadgwick);
LogFileSD.print(";");
LogFileSD.print(yawMadgwick);
LogFileSD.print(";");

LogFileSD.print(valueRawAdcCh1);
LogFileSD.print(";");
LogFileSD.print(valueRawAdcCh2);
LogFileSD.print(";");

LogFileSD.print(MPX2010valueKpa);
LogFileSD.print(";");
LogFileSD.print(MPXHZ6115valueKpa);
LogFileSD.print(";");
LogFileSD.print(MPX2010valueKpaKalman);
LogFileSD.print(";");
LogFileSD.print(MPXHZ6115valueKpaKalman);
LogFileSD.print(";");
LogFileSD.print(AltitudeFromPressure);
LogFileSD.print(";");
LogFileSD.print(WindSpeedFromPressure);

LogFileSD.println(";");

LogFileSD.close();
//Serial.println("SD OK");
} else {
//Serial.println("SD NOT OK");
}
}


