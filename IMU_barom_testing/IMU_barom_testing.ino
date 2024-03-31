#include <Wire.h>
#include <Adafruit_GPS.h>  // GPS   Softwareserial
#include <SoftwareSerial.h>
#include <Arduino_LSM6DSOX.h>  // IMUs   could be SPI or I2C, don't rlly know; LSM6DS library uses I2C, LSM6DSOX uses SPI
#include <LPS22HBSensor.h>  // barometer   I2C
#include <SPI.h>  // something
#include <Arduino.h>


// IMU ports
// chip selectors for the two IMUs
// CS01 is chip selector for IMU1
// CS02 is chip selector for IMU2
#define CS1 37
#define CS2 36

#define IMUMISO 12
#define IMUMOSI 11

// GPS ports
#define GPSRx 28
#define GPSTx 29

// barometer ports
#define baromSDA 2   // could also be A2, idk
#define baromSCL 3   // could also be A3, idk

SoftwareSerial mySerial(GPSTx, GPSRx);
Adafruit_GPS GPS(&mySerial);


LPS22HBSensor barom(&Wire);

void switch1() {
  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, LOW);
}

void switch2() {
  digitalWrite(CS1, LOW);
  digitalWrite(CS2, HIGH);
}


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  delay(2000);

  // setting up GPS signaling
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(1000);

  // setting up IMUs
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);

  // initializing IMU1
  switch1(); if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU 1!");
  }
  Serial.println("end of IMU1 init");
  // Serial.print("IMU1 Accelerometer sample rate = ");  Serial.print(IMU.accelerationSampleRate());  Serial.println(" Hz");
  // Serial.print("IMU1 Gyroscope sample rate = ");      Serial.print(IMU.gyroscopeSampleRate());      Serial.println(" Hz");
  // initializing IMU2
  switch2(); if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU 2!");
  }

  Serial.println("end of IMU2 init");
  // Serial.print("IMU2 Accelerometer sample rate = ");  Serial.print(IMU.accelerationSampleRate());  Serial.println(" Hz");
  // Serial.print("IMU2 Gyroscope sample rate = ");      Serial.print(IMU.gyroscopeSampleRate());      Serial.println(" Hz");

  // barometer init 
  Wire.begin();
  barom.begin();
  barom.Enable();


}

void loop() {
  // put your main code here, to run repeatedly:
  // GPS values
  float lat, lon, alt, speed;
  // IMU values
  float xg1, yg1, zg1, xa1, ya1, za1;
  float xg2, yg2, zg2, xa2, ya2, za2;
  float xg, yg, zg, xa, ya, za;
  float p, t;

  lat = GPS.latitude;
  lon = GPS.longitude; 
  alt = GPS.altitude;
  speed = GPS.speed / 1.94384;

  switch1(); IMU.readAcceleration(xa1, ya1, za1); IMU.readGyroscope(xg1, yg1, zg1);
  switch2(); IMU.readAcceleration(xa2, ya2, za2); IMU.readGyroscope(xg2, yg2, zg2);

  xg = (xg1 + xg2)/2;
  yg = (yg1 + yg2)/2;
  zg = (zg1 + zg2)/2;
  xa = (xa1 + xa2)/2;
  ya = (ya1 + ya2)/2;
  za = (za1 + za2)/2;

  barom.GetPressure(&p);
  barom.GetTemperature(&t);

  Serial.println("Latitude \t Longitude \t Altitude \t xAccel \t yAccel \t zAccel \t xRot \t yRot \t zRot \t Pressure \t Temp");
  Serial.print(lat); Serial.print("\t\t"); Serial.print(lon); Serial.print("\t\t"); Serial.print(alt); Serial.print("\t\t");
  Serial.print(xa); Serial.print("\t"); Serial.print(ya); Serial.print("\t"); Serial.print(za); Serial.print("\t");
  Serial.print(xg); Serial.print("\t"); Serial.print(yg); Serial.print("\t"); Serial.print(zg); Serial.print("\t");
  Serial.print(p); Serial.print("\t");
  Serial.print(t); Serial.println();

  delay(50);
}








