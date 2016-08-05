
// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11

// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, 
                                      LIS3DH_CLK);

// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

// I2C
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();

float accelX, accelY, accelZ;
float prevAccelX, prevAccelY, prevAccelZ;
float deltaX, deltaY, deltaZ;
float bumpVectorLength;

void setup(void) 
{
#ifndef ESP8266
  //while (!Serial);     // will pause Zero, Leonardo, etc until serial 
                         // console opens
#endif

  Serial.begin(9600);
  //Serial.println("LIS3DH test!");
  
  //change this to 0x19 for alternative i2c address
  if (! lis.begin(0x18)) 
  {       
    Serial.println("Couldnt start");
    while (1);
  }

  //set the range to +/- 16g
  lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
}

void loop() 
{


  if(Serial.available() > 0)
  {
    //1str = Serial.readStringUntil('\n');
    kFilteringFactor= Serial.parseFloat();
    Serial.println(kFilteringFactor);
  }

  
  lis.read();      // get X Y and Z data at once

  sensors_event_t event; 
  lis.getEvent(&event);

  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;


  accelX = lis.x - ( ( lis.x * kFilteringFactor ) + ( accelX * ( 1.0 - 
           kFilteringFactor ) ) );
  accelY = lis.y - ( ( lis.y * kFilteringFactor ) + ( accelY * ( 1.0 - 
           kFilteringFactor ) ) );
  accelZ = lis.z - ( ( lis.z * kFilteringFactor ) + ( accelZ * ( 1.0 - 
           kFilteringFactor ) ) );

  deltaX = abs( ( accelX - prevAccelX ) );
  deltaY = abs( ( accelY - prevAccelY ) );
  deltaZ = abs( ( accelZ - prevAccelZ ) );

  bumpVectorLength = sqrt( sq( deltaX ) + sq( deltaY ) + sq( deltaZ ) );

  Serial.print(deltaX);
  Serial.print(" ");
  Serial.print(deltaY);
  Serial.print(" ");
  Serial.println(deltaZ);

  delay(1); 
}
