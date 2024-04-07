#include <Wire.h>
#include <Adafruit_GPS.h>  // GPS   software serial
#include <SoftwareSerial.h>
#include <Adafruit_LSM6DSOX.h>  // IMUs   SPI
#include <LPS22HBSensor.h>  // barometer
#include <SPI.h>  // something
#include <RH_RF95.h>  // radio
#include <TeensyThreads.h>
#include <SD.h>  // SD library
#include <Arduino.h>
#include <stdint.h>
#include <SoftwareSerial.h>

#define LED_R 41
#define LED_G 40
#define LED_B 39
#define BUZZ 2

#define GPSRx 28
#define GPSTx 29

#define I2C2_SCL  A3
#define I2C2_SDA  A2
// Barometer components
TwoWire dev_i2c = Wire2;
LPS22HBSensor barom(&dev_i2c);

// change these values except the frequency
#define RadioCS 10      
#define RadioRST 9
#define RadioINT 2
#define RadioFREQ 915.0

// IMU variables
// software-SPI mode
#define LSM_SCK
#define LSM_MISO 12
#define LSM_MOSI 11

RH_RF95 radio(RadioCS, RadioINT);

  Adafruit_LSM6DSOX imu1;
  Adafruit_LSM6DSOX imu2;

SoftwareSerial mySerial(GPSTx, GPSRx);
Adafruit_GPS gps(&mySerial);

void setup() {

  Serial.begin(9600);
  while (!Serial);
  Serial.println("hello");
  // put your setup code here, to run once:
  // stuff to set up:
    // RADIO - HopeRF RFM95W - LoRaNow library installed
      // 0-> CSI
      // 1-> MISO1
      // A13 - 27 -> SCK1
      // A12 - 26 -> MOSI1
    // GPS - Adafruit Ultimate GPS - library downloaded
      // 28 -> Rx
      // 29 -> Tx
    // IMUs - lsm6dsox - installed
      // 13 -> CLK0
      // CS3 - 37 -> CS01
      // CS2 - 36 -> CS02
      // MISO - 12 -> MISO0
      // MOSI - 11 -> MOSI0
    // IMU1 - lsm6dsox - installed
      // OUT1C - 9 -> INT2
      // CS1 - 10 -> INT1
    // IMU2 - lsm6dsox - installed
      // A11 - 25 -> INT1
      // A10 - 24 -> INT2
    // BAROM - lps22hb - STM LPS22HB installed
      // 15 - A1 -> INT
      // 16 - A2 -> SDA1
      // 17 - A3 -> SCL1
    // BUZZ - 
      // OUT2 - 2 -> MCU_BUZZ
    pinMode(BUZZ, OUTPUT);
    // RGB LED
      // A17 - 41 -> MCU LED B
    pinMode(LED_B, OUTPUT);
      // A16 - 40 -> MCU LED G
    pinMode(LED_G, OUTPUT);
      // A15 - 39 -> MCU LED R
    pinMode(LED_R, OUTPUT);


  // setting up sensor settings
  Serial.begin(9600);
  while (!Serial);

  // radio init
  if (!radio.init()) {Serial.println("Radio init failed");}
  if (!radio.setFrequency(RadioFREQ)) {Serial.println("Radio frequency setting failed");}

  // IMUs init
  if (!imu1.begin_I2C()) {Serial.println("IMU1 init failed");}
  if (!imu2.begin_I2C()) {Serial.println("IMU2 init failed");}

  Serial.println("IMU1 Sampling Info: ");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(imu1.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(imu1.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("IMU2 Sampling Info: ");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(imu2.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(imu1.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();

  gps.begin(9600);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  Serial.println("GPS init succeeded");

  barom.begin();
  barom.Enable();
  // test pressure reading
  float p;
  barom.GetPressure(&p);
  if (!p) {Serial.println("Barometer init failed");}

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }

  // Create a new file on the SD card
  File dataFile;
  dataFile = SD.open("dataFC.txt", FILE_WRITE);


  //Threading setup
  sampleBarometer();
  // threads.addThread(sampler);
  Serial.println("Finished sampler");
  // threads.addThread(transmitter);


}

// Sampling Data Arrays
// wiped every second

const int sendingFrequency = 10; // 10 Hz

float gps_data[4][sendingFrequency];    // gps --> [lat, lon, alt, speed]
float imu1_data[6][sendingFrequency];   // gyro and accel data 1 --> [xg, yg, zg, xa, ya, za]
float imu2_data[6][sendingFrequency];   // gyro and accel data 2 --> [xg, yg, zg, xa, ya, za]
float barom_data[2][sendingFrequency];  // barometer --> [pressure, temperature]

// Transmission data arrays
volatile float gps_transmit[4][sendingFrequency];
volatile float imu1_transmit[6][sendingFrequency];
volatile float imu2_transmit[6][sendingFrequency];
volatile float barom_transmit[2][sendingFrequency];

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

void sampleBarometer() {
  Serial.println("Sampling barometer");
  float p, t;
  uint32_t start = millis();
  while (millis() - start < 100000) {
    barom.GetPressure(&p);
    barom.GetTemperature(&t);
    // barom_data[0][(int)(timer/10)] = p;
    // barom_data[1][(int)(timer/10)] = t;
    Serial.print(p); Serial.print(" ... "); Serial.println(t);
  }
}


// writing to data arrays at 100 Hz
void sampler() {
  Serial.println("Started sampler");
  uint32_t timer = millis();
  Serial.println(timer);
  while (millis() - timer < 10000) {
    if (millis() - timer % 10 <= 2 && millis() - timer >= 8) {
      float xg1, yg1, zg1, xg2, yg2, zg2, xa1, ya1, za1, xa2, ya2, za2;
      Serial.print("Sampling ... ");
      if (imu1.gyroscopeAvailable()) {
        imu1.readGyroscope(xg1, yg1, zg1);
        imu1_data[0][(int)(timer/10)] = xg1;
        imu1_data[1][(int)(timer/10)] = yg1;
        imu1_data[2][(int)(timer/10)] = zg1;
      }
      if (imu2.gyroscopeAvailable()) {
        imu2.readGyroscope(xg2, yg2, zg2);
        imu2_data[0][(int)(timer/10)] = xg2;
        imu2_data[1][(int)(timer/10)] = yg2;
        imu2_data[2][(int)(timer/10)] = zg2;
      }
      if (imu1.accelerationAvailable()) {
        imu1.readAcceleration(xa1, ya1, za1);
        imu1_data[3][(int)(timer/10)] = xa1;
        imu1_data[4][(int)(timer/10)] = ya1;
        imu1_data[5][(int)(timer/10)] = za1;
      }
      if (imu2.accelerationAvailable()) {
        imu2.readAcceleration(xa2, ya2, za2);
        imu2_data[3][(int)(timer/10)] = xa2;
        imu2_data[4][(int)(timer/10)] = ya2;
        imu2_data[5][(int)(timer/10)] = za2;
      }
      if (gps.fix) {
        gps_data[0][(int)(timer/10)] = gps.latitude;
        gps_data[1][(int)(timer/10)] = gps.longitude; 
        gps_data[2][(int)(timer/10)] = gps.altitude;
        gps_data[3][(int)(timer/10)] = gps.speed;
      }
      float p, t;
      Serial.print(" Got to barometer sampling ... ");
      barom.GetPressure(&p);
      barom.GetTemperature(&t);
      barom_data[0][(int)(timer/10)] = p;
      barom_data[1][(int)(timer/10)] = t;
      Serial.print(p); Serial.print(" ... "); Serial.println(t);
      Serial.print("Data sampled at ");
      Serial.println(timer);
    }
    if (millis() - timer % 1000 == 0) {
      // copy arrays over to transmission arrays
      memcpy(gps_transmit, gps_data, sizeof(gps_data));
      memcpy(imu1_transmit, imu1_data, sizeof(imu1_data));
      memcpy(imu2_transmit, imu2_data, sizeof(imu2_data));
      memcpy(barom_transmit, barom_data, sizeof(barom_data));
      Serial.print("Data copied at ");
      Serial.println(timer);
    }
    
  }
  
}

void sendWithCrc(void* data, size_t len) {
  uint16_t crc = crc16_ccitt((uint8_t*) data, len);
  radio.send((uint8_t*) data, len);
  radio.send((uint8_t*)&crc, sizeof(crc));
}


void transmitter() {    // send to radio and SD card
  uint32_t timer = millis();
  // send transmission arrays to SD card
  while (timer < 10000) {
    if (timer % 1000 == 0) {
      digitalWrite(LED_G, HIGH);
      File dataFile;
      dataFile = SD.open("data.txt", FILE_WRITE);
      if (dataFile) {
        // Write data to the file
        // Example: Write GPS data
        for (int i = 0; i < 100; i++) {
          dataFile.print(gps_transmit[0][i]);dataFile.print(", ");
          dataFile.print(gps_transmit[1][i]);dataFile.print(", ");
          dataFile.print(gps_transmit[2][i]);dataFile.print(", ");
          dataFile.print(gps_transmit[3][i]);dataFile.print(", ");
          dataFile.print(imu1_transmit[0][i]);dataFile.print(", ");
          dataFile.print(imu1_transmit[1][i]);dataFile.print(", ");
          dataFile.print(imu1_transmit[2][i]);dataFile.print(", ");
          dataFile.print(imu1_transmit[3][i]);dataFile.print(", ");
          dataFile.print(imu1_transmit[4][i]);dataFile.print(", ");
          dataFile.print(imu1_transmit[5][i]);dataFile.print(", ");
          dataFile.print(imu2_transmit[0][i]);dataFile.print(", ");
          dataFile.print(imu2_transmit[1][i]);dataFile.print(", ");
          dataFile.print(imu2_transmit[2][i]);dataFile.print(", ");
          dataFile.print(imu2_transmit[3][i]);dataFile.print(", ");
          dataFile.print(imu2_transmit[4][i]);dataFile.print(", ");
          dataFile.print(imu2_transmit[5][i]);dataFile.print(", ");
          dataFile.print(barom_transmit[0][i]);dataFile.print(", ");
          dataFile.print(barom_transmit[1][i]);dataFile.println();
        }

        // Close the file
        dataFile.close();
      }
      sendWithCrc(gps_transmit, sizeof(gps_transmit));
      sendWithCrc(imu1_transmit, sizeof(imu1_transmit));
      sendWithCrc(imu2_transmit, sizeof(imu2_transmit));
      sendWithCrc(barom_transmit, sizeof(barom_transmit));

      // send transmission arrays to 
      Serial.print("Transmitting at timer = ");
      Serial.println(timer);
      // letting the led stay on for a bit longer to signal that transmission has happened
      delay(200);
      // turn off LED
      digitalWrite(LED_G, LOW);
    }
    delay(2);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
