/* PERUN ROCKETS TELEMETRY v0.2 
 * Original sketch by Kamlo 2017
 */

import processing.serial.*;

Serial myPort;
PShape s;
PImage img;
PFont Font1;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float Altitude = 0.0;
float Speed = 0.0;
float Temperature = 0.0;
float SecondRun = 0.0;
int STAT_STARTUP, STAT_LIFTOF, STAT_DEPLOY_P, STAT_FINISH;

void setup()
{
  size(600, 500, P3D);

  // if you have only ONE serial port active
  myPort = new Serial(this, Serial.list()[0], 500000); // if you have only ONE serial port active

  // if you know the serial port name
  //myPort = new Serial(this, "COM5:", 9600);                    // Windows
  //myPort = new Serial(this, "/dev/ttyACM0", 9600);             // Linux
  //myPort = new Serial(this, "/dev/cu.usbmodem1217321", 9600);  // Mac

  img = loadImage("perun.png");
  s = loadShape("AIM120D.obj");
  
  Font1 = createFont("Consolas Bold", 22);

  textSize(16); // set text size
  //textMode(SHAPE); // set text mode to shape

}

void draw()
{
  serialEvent();  // read and parse incoming serial message
  //background(255); // set background to white
  background(104, 151, 229);
  lights();

    image(img, 0, 0, img.width/2, img.height/2);
    

    //fill(204, 102, 0); 
    textFont(Font1);
    text("T+:"+(int)SecondRun+"s",width-240,60);
    text("Altitude:"+Altitude+"m",width-240,100);
    text("Speed:"+Speed+"m/s",width-240,125);
    text("Temp:"+Temperature+"°c",width-240,150);

    if(STAT_STARTUP == 1) {
    ColorStatus();
    }
    text("STARTUP → ",20,height-40);
    fill(255,255,255);
    
    if(STAT_LIFTOF == 1) {
    ColorStatus();
    }
    text("LIFTOFF → ",143,height-40);
    fill(255,255,255);
    
    if(STAT_DEPLOY_P == 1) {
    ColorStatus();
    }
    text("DEPLOY PARACHUTE → ",263,height-40);
    fill(255,255,255);
    
    if(STAT_FINISH == 1) {
    ColorStatus();
    }
    text("FINISH",493,height-40);
    fill(255,255,255);

  translate(width/2, height/2); // set position to centre

  pushMatrix(); // begin object

  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(pitch));
  float s2 = sin(radians(pitch));
  float c3 = cos(radians(yaw));
  float s3 = sin(radians(yaw));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  drawArduino();
  


  popMatrix(); // end of object

  // Print values to console
  print(roll);
  print("\t");
  print(pitch);
  print("\t");
  print(yaw);
  println();
}

void serialEvent()
{
  int newLine = 13; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    if (message != null) {
      String[] list = split(trim(message), " ");
      if (list.length >= 4 && list[0].equals("Telemetry:")) {
        yaw = float(list[1]); // convert to float yaw
        pitch = float(list[2]); // convert to float pitch
        roll = float(list[3]); // convert to float roll
        
        Altitude = float(list[4]);
        Speed = float(list[5]);
        Temperature = float(list[6]);
        SecondRun = float(list[7]);
        
        STAT_STARTUP = int(list[8]);
        STAT_LIFTOF = int(list[9]);
        STAT_DEPLOY_P = int(list[10]);
        STAT_FINISH = int(list[11]);
        
      }
    }
  } while (message != null);
}

void drawArduino()
{
  
  
  scale(5);
  shape(s);
  
}

void ColorStatus() {
  fill(36,113,163);
}