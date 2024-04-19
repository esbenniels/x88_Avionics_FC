// GPS
#include <Adafruit_GPS.h>  // GPS   software serial
#include <SoftwareSerial.h>

#define GPSRx 7
#define GPSTx 8

SoftwareSerial mySerial(GPSTx, GPSRx);
Adafruit_GPS gps(&mySerial);

// I2C stuff
#include "Wire.h"
#define IMU_I2C_ADDR 0x6A
#define BAROM_I2C_ADDR 0x5D
// imu
#include <Adafruit_LSM6DS3TRC.h> // IMU - I2C comms
Adafruit_LSM6DS3TRC imu1;  // interact with I2C address 0x6A
#define imu1_CS 40


// barom
#include <LPS22HBSensor.h> // barometer - I2C comms
LPS22HBSensor barom(&Wire); // interact with I2C address 0x5C

// radio
#include <LoRa.h> // LoRa radio
#define RADIO_CS 10
#define RADIO_MISO 12
#define RADIO_MOSI 11
#define RADIO_SCK 13

// SD Card
#include <SD.h>

// SPI
#include <SPI.h>

File dataFile;

void setup() {
  Serial.begin(9600);

  // IMU setup
  Wire.beginTransmission(IMU_I2C_ADDR);
  if (!imu1.begin_I2C()) {
    Serial.println("IMU initialization failed");
    while(1);
  };

  imu1.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  imu1.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

  pinMode(imu1_CS, OUTPUT);
  Wire.endTransmission();

  // Barometer setup
  Wire.beginTransmission(BAROM_I2C_ADDR);
  barom.begin();
  barom.Enable();
  barom.SetODR(100);
  Wire.endTransmission();
  // pinMode(BAROM_CS, OUTPUT);

  // GPS setup
  if (!gps.begin(9600)) {
    Serial.println("GPS initialization failed");
    while(1);
  };
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  Serial.println("GPS init succeeded");

  // LoRa setup
  LoRa.setPins(RADIO_CS);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");  
  }

  // SD Card setup
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while(1);
  }

  dataFile = SD.open("dataFC.txt", FILE_WRITE);
  dataFile.println("transmissionTime,pressure,temperature,lat,lon,alt,speed,accel1.x,accel1.y,accel1.z,gyro1.x,gyro1.y,gyro1.z//checksum");
  delay(500);
  dataFile.close();

  // while(1);

  // dataFile = SD.open("dataGS.txt", FILE_WRITE);

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

sensors_event_t accel1, gyro1, temp1;

void loop() {
  // Serial.println("loop");
  digitalWrite(imu1_CS, HIGH);
  // digitalWrite(BAROM_CS, HIGH);
  Wire.beginTransmission(BAROM_I2C_ADDR);
  barom.GetPressure(&pressure);
  barom.GetTemperature(&temperature);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_I2C_ADDR);
  if (imu1.accelerationAvailable() && imu1.gyroscopeAvailable()) {
    imu1.getEvent(&accel1, &gyro1, &temp1);
  } else {
    Serial.println("Accel and gyro not available");
  }
  Wire.endTransmission();

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
    "," + String(gyro1.gyro.z);

  Serial.println(dataString);

  uint16_t dataChecksum = crc16_ccitt((uint8_t*) dataString.c_str(), dataString.length());
  // Serial.println("Calculated checksum");

  // send data packet
  LoRa.beginPacket();
  // Serial.println("Began packet");

  LoRa.print(dataString); LoRa.print("//"); LoRa.print(dataChecksum); LoRa.print("|");
  // Serial.println("Printed to radio");

  LoRa.endPacket();

  // Serial.println("Ended packet");


  // print to datafile but this time separated by commas
  dataFile = SD.open("dataFC.txt", FILE_WRITE);
  dataFile.print(dataString); dataFile.print("//"); dataFile.print(dataChecksum); dataFile.println("|");

  // while (micros() - lastTransmissionTime < 1000000 / sendingFrequency) {Serial.println("Waiting for next writeTime");}

  // lastTransmissionTime = micros();

  // delay(2000);

}