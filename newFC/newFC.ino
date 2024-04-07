// GPS
#include <Adafruit_GPS.h>  // GPS   software serial
#include <SoftwareSerial.h>

#define GPSRx 7
#define GPSTx 8

SoftwareSerial mySerial(GPSTx, GPSRx);
Adafruit_GPS gps(&mySerial);

// I2C stuff
#include "Wire.h"
// imu
#include <Adafruit_LSM6DSOX.h> // IMU - I2C comms
Adafruit_LSM6DSOX sox1;  // interact through I2C channel 1 - Wire1 - pins 16 and 17
#define SOX1_CS 40
// Adafruit_LSM6DSOX sox2;  // interact through I2C channel 1 - Wire1 - pins 18 and 19


// barom
#include <LPS22HBSensor.h> // barometer - I2C comms
LPS22HBSensor barom(&Wire2); // put this on Wire2 ports - pins 24 and 25
#define BAROM_CS 41

// radio
#include <LoRa.h> // LoRa radio
const int RADIO_CS = 0;
#define RADIO_MISO 1
#define RADIO_MOSI 26

// SD Card
#include <SD.h>

// SPI
#include <SPI.h>

File dataFile;

void setup() {
  Serial.begin(9600);

  // IMU setup
  Wire1.begin();
  if (!sox1.begin_I2C()) {
    Serial.println("IMU initialization failed");
  };
  pinMode(SOX1_CS, OUTPUT);

  // Barometer setup
  Wire2.begin();
  barom.begin();
  barom.Enable();
  barom.SetODR(10);
  pinMode(BAROM_CS, OUTPUT);

  // GPS setup
  if (!gps.begin(9600)) {
    Serial.println("GPS initialization failed");
  };
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  Serial.println("GPS init succeeded");

  // LoRa setup
  LoRa.setPins(RADIO_CS);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");  
  }

  // SD Card setup
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
  }

  dataFile = SD.open("dataFC.txt", FILE_WRITE);
  dataFile.println("transmissionTime,pressure,temperature,lat,lon,alt,speed,accel1.x,accel1.y,accel1.z,gyro1.x,gyro1.y,gyro1.z,accel2.x,accel2.y,accel2.z,gyro2.x,gyro2.y,gyro2.z//checksum");
}

const int sendingFrequency = 10; // 10 Hz

float gps_data[4][sendingFrequency];    // gps --> [lat, lon, alt, speed]
float imu1_data[6][sendingFrequency];   // gyro and accel data 1 --> [xg, yg, zg, xa, ya, za]
float imu2_data[6][sendingFrequency];   // gyro and accel data 2 --> [xg, yg, zg, xa, ya, za]
float barom_data[2][sendingFrequency];  // barometer --> [pressure, temperature]


// checksum creator
uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// checksum sending function
// void sendWithCrc(void* data, size_t len) {
//   uint16_t crc = crc16_ccitt((uint8_t*) data, len);
//   radio.send((uint8_t*) data, len);
//   radio.send((uint8_t*)&crc, sizeof(crc));
// }

uint32_t globalTime = micros();

float lastTransmissionTime = micros();

float pressure, temperature, lat, lon, alt, speed;

sensors_event_t accel1, gyro1, temp1, accel2, gyro2, temp2;

void loop() {
  
  digitalWrite(SOX1_CS, HIGH);
  digitalWrite(BAROM_CS, HIGH);

  barom.GetPressure(&pressure);
  barom.GetTemperature(&temperature);

  if (sox1.accelerationAvailable() && sox1.gyroscopeAvailable()) {
    sox1.getEvent(&accel1, &gyro1, &temp1);
  } else {
    Serial.println("Accel and gyro not available");
  }

  if (gps.fix) {
    lat = gps.latitude;
    lon = gps.longitude; 
    alt = gps.altitude;
    speed = gps.speed;
  }

  float transmissionTime = micros() - globalTime;
  transmissionTime = round(transmissionTime * 1000) / 1000;
  
  String dataString = String(transmissionTime) + "," + String(pressure) + "," + String(temperature) + 
    "," + String(lat) + "," + String(lon) + "," + String(alt) + 
    "," + String(speed) + "," + String(accel1.acceleration.x) + "," + String(accel1.acceleration.y) + 
    "," + String(accel1.acceleration.z) + "," + String(gyro1.gyro.x) + "," + String(gyro1.gyro.y) + 
    "," + String(gyro1.gyro.z) + "," + String(accel2.acceleration.x) + "," + String(accel2.acceleration.y) + 
    "," + String(accel2.acceleration.z) + "," + String(gyro2.gyro.x) + "," + String(gyro2.gyro.y) + 
    "," + String(gyro2.gyro.z);

  Serial.println(dataString);

  uint16_t dataChecksum = crc16_ccitt((uint8_t*) dataString.c_str(), dataString.length());

  // send data packet
  LoRa.beginPacket();

  LoRa.print(dataString); LoRa.print("//"); LoRa.print(dataChecksum);

  LoRa.endPacket();


  // print to datafile but this time separated by commas

  dataFile.print(dataString); dataFile.print("//"); dataFile.print(dataChecksum); dataFile.println();

  while (micros() - lastTransmissionTime < 1000000 / sendingFrequency) {Serial.println("Waiting for next writeTime");}

  lastTransmissionTime = micros();

}