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
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   //#define Serial SerialUSB
#endif

  float accelX, accelY, accelZ;
  float prevAccelX, prevAccelY, prevAccelZ;
  float deltaX, deltaY, deltaZ;
  float bumpVectorLength;

  float kFilteringFactor;// = .75;
  float sensitivityThreshold = 3.0;

  int x;
  String str;

void setup(void) {
#ifndef ESP8266
  //while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  kFilteringFactor = .85;

  Serial.begin(9600);
  //Serial.println("LIS3DH test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  //Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
  
  //Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  //Serial.println("G");


}

void loop() {


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

  //accelX = event.acceleration.x - ( ( event.acceleration.x * kFilteringFactor ) + ( accelX * ( 1.0 - kFilteringFactor ) ) );
  //accelY = event.acceleration.y - ( ( event.acceleration.y * kFilteringFactor ) + ( accelY * ( 1.0 - kFilteringFactor ) ) );
  //accelZ = event.acceleration.z - ( ( event.acceleration.z * kFilteringFactor ) + ( accelZ * ( 1.0 - kFilteringFactor ) ) );

  accelX = lis.x - ( ( lis.x * kFilteringFactor ) + ( accelX * ( 1.0 - kFilteringFactor ) ) );
  accelY = lis.y - ( ( lis.y * kFilteringFactor ) + ( accelY * ( 1.0 - kFilteringFactor ) ) );
  accelZ = lis.z - ( ( lis.z * kFilteringFactor ) + ( accelZ * ( 1.0 - kFilteringFactor ) ) );

  deltaX = abs( ( accelX - prevAccelX ) );
  deltaY = abs( ( accelY - prevAccelY ) );
  deltaZ = abs( ( accelZ - prevAccelZ ) );

  bumpVectorLength = sqrt( sq( deltaX ) + sq( deltaY ) + sq( deltaZ ) );
/*
  if( bumpVectorLength > sensitivityThreshold ){
 
    Serial.print("Bump Detected! \tX: "); Serial.print(deltaX); Serial.print("\tY: "); Serial.print(deltaY); Serial.print("\tZ: "); Serial.print(deltaZ); Serial.print("\tLength: "); Serial.println(bumpVectorLength);
  }
  else{
    Serial.print("--\t--");
  }
  */
  Serial.print(deltaX);
  Serial.print(" ");
  Serial.print(deltaY);
  Serial.print(" ");
  Serial.println(deltaZ);

/*
  
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 

  /* Or....get a new sensor event, normalized */ 
  //sensors_event_t event; 
  //lis.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  //Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  //Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
 // Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  //Serial.print(" m/s^2 ");
  //Serial.print(" \tVal: "); Serial.println( sqrt( sq( event.acceleration.x )+ sq( event.acceleration.y ) + sq( event.acceleration.z ) ) );

  //Serial.print(" ");
 
  delay(1); 
}
