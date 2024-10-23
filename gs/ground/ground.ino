#include <arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SPI.h>  // something  
#include <SD.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"
#include <cstdio>
#include <LoRa.h>
// #include <avr/io.h>
// #include <avr/interrupt.h>


// change these values except the frequency
#define RADMISO 12
#define RADMOSI 11
#define RADSCK 13
const int RADIOCS = 10;

#define DISPMISO 12
#define DISPMOSI 11
#define DISPSCK 13
const int DISPCS = 9;

#define BUZZ 2

// Connect SCLK to UNO Digital #13 (Hardware SPI clock)
// Connect MISO to UNO Digital #12 (Hardware SPI MISO)
// Connect MOSI to UNO Digital #11 (Hardware SPI MOSI)
#define RA8875_INT 41
#define RA8875_RESET 40

SPISettings settingsRADIO(2000000, MSBFIRST, SPI_MODE1);
SPISettings settingsDISP(16000000, LSBFIRST, SPI_MODE3);


const int INT_PIN = 3;

// dorm - waiting for interrupt
// wait - waiting for FC ack
// rec - FC has acknowledged and both stations are recording
volatile String FCRecStatus = "dorm"; 
const int monitorLED = 32;

bool sentFlag = 0;

void sendBEGIN() {
  if (sentFlag == 0) {
    LoRa.beginPacket();
    LoRa.print("BEGIN");
    LoRa.endPacket();
    Serial.println("Sent BEGIN to FC");
    FCRecStatus = "wait";
    digitalWrite(monitorLED, HIGH);
    delay(100);
    digitalWrite(monitorLED, LOW);
    sentFlag = 1;
  }
  
}


//Adafruit_RA8875 tft = Adafruit_RA8875(DISPCS, RA8875_RESET);

// CRC checksum function
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




// define variables for the X and Y coordinates of the various flight parameters to be displayed
int flightTimeTitleX = 100;
int flightTimeTitleY = 2;
int signalTitleX = 47;
int signalTitleY = 42;
int speedTitleX = 280;
int speedTitleY = 42;
int posTitleX = 460;
int posTitleY = 42;
int latTitleX = 415;
int latTitleY = 100;
int lonTitleX = 415;
int lonTitleY = 150;
int tempTitleX = 648;
int tempTitleY = 42;
int pressTitleX = 70;
int pressTitleY = 262;
int gyroTitleX = 280;
int gyroTitleY = 262;
int accelTitleX = 475;
int accelTitleY = 262;
int altTitleX = 663;
int altTitleY = 262;
int xgyroX = 218;
int xgyroY = 320;
int ygyroX = 218;
int ygyroY = 370;
int zgyroX = 218;
int zgyroY = 420;
int xaccelX = 415;
int xaccelY = 320;
int yaccelX = 415;
int yaccelY = 370;
int zaccelX = 415;
int zaccelY = 420;
int signalIndicatorX = 21;
int signalIndicatorY = 210;

int signalBoxBounds[4] = {16, 65, 170, 180};
int speedBoxBounds[4] = {213, 65, 170, 180};
int posBoxBounds[4] = {508, 80, 50, 100};
int tempBoxBounds[4] = {607, 65, 170, 180};

int pressBoxBounds[4] = {16, 285, 170, 180};
int gyroBoxBounds[4] = {213, 285, 170, 180};
int accelBoxBounds[4] = {410, 285, 170, 180};
int altBoxBounds[4] = {607, 285, 170, 180};


File dataFile;

void startSPI(const int CS, SPISettings settings) {
  digitalWrite(CS, LOW);
  SPI.beginTransaction(settings);
}

void endSPI(const int CS) {
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

float lastReception = millis();

float gps[4] = {0, 0, 0, 0};
float imu[6] = {0, 0, 0, 0, 0, 0};
float barom[2] = {0, 0};

Adafruit_RA8875 tft = Adafruit_RA8875(DISPCS, RA8875_RESET);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Made it to setup");
  
  pinMode(INT_PIN, INPUT_PULLUP);
  Serial.println("Defined INT_PIN");
  attachInterrupt(digitalPinToInterrupt(INT_PIN), sendBEGIN, RISING);
  Serial.println("Established Interrupt");

  pinMode(RADIOCS, OUTPUT);
  pinMode(DISPCS, OUTPUT);
  // pinMode(monitorLED, OUTPUT);

  SPI.begin();

  startSPI(RADIOCS, settingsRADIO);

  LoRa.setPins(RADIOCS);

  // radio init
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
  } else {
    Serial.println("Starting LoRa succceeded");
  }

  endSPI(RADIOCS);

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }


  dataFile = SD.open("dataGS.txt", FILE_WRITE);
  dataFile.println("transmissionTime,pressure,temperature,latitude,longitude,altitude,speed,pitch,yaw,roll,signalStrength//checksum");
  delay(500);
  dataFile.close();

  
  /* Initialize the display using 'RA8875_480x80', 'RA8875_480x128', 'RA8875_480x272' or 'RA8875_800x480' */
  startSPI(DISPCS, settingsDISP);

  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 Not Found!");
  } else {
    Serial.println("Found RA8875");
  }

  tft.displayOn(true);
  tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  tft.PWM1out(255);

  // With hardware accelleration this is instant
  // tft.fillScreen(RA8875_WHITE);

  Serial.print("(");
  Serial.print(tft.width());
  Serial.print(", ");
  Serial.print(tft.height());
  Serial.println(")");
  tft.graphicsMode();                 // go back to graphics mode
  tft.fillScreen(RA8875_BLACK);
  tft.graphicsMode();
  bmpDraw("MRPL_MissionPatch_24.bmp", 0, 0);

  delay(1500);

  tft.fillScreen(RA8875_BLACK);

  tft.drawRect(12, 40, 185, 210, RA8875_WHITE);
  tft.drawRect(209, 40, 185, 210, RA8875_WHITE);
  tft.drawRect(406, 40, 185, 210, RA8875_WHITE);
  tft.drawRect(603, 40, 185, 210, RA8875_WHITE);
  tft.drawRect(12, 260, 185, 210, RA8875_WHITE);
  tft.drawRect(209, 260, 185, 210, RA8875_WHITE);
  tft.drawRect(406, 260, 185, 210, RA8875_WHITE);
  tft.drawRect(603, 260, 185, 210, RA8875_WHITE);

  Serial.println("Finished Drawing Rectangles");

  tft.textMode();

  tft.textSetCursor(flightTimeTitleX, flightTimeTitleY);

  tft.textEnlarge(1);

  tft.textTransparent(RA8875_WHITE);
  tft.textWrite("FLIGHT TIME: 00:00.00 s");
  // signal strength box
  tft.textSetCursor(signalTitleX, signalTitleY);
  tft.textEnlarge(0);
  tft.textWrite("Signal Strength");
  // altitude box
  tft.textSetCursor(speedTitleX, speedTitleY);
  tft.textWrite("Speed");
  // GPS box
  tft.textSetCursor(posTitleX, posTitleY);
  tft.textWrite("Position");
  tft.textEnlarge(1);
  tft.textSetCursor(latTitleX, latTitleY);
  tft.textWrite("Lat: ");
  tft.textSetCursor(lonTitleX, lonTitleY);
  tft.textWrite("Lon: ");
  // temperature box
  tft.textEnlarge(0);
  tft.textSetCursor(tempTitleX, tempTitleY);
  tft.textWrite("Temperature");
  // pressure box
  tft.textSetCursor(pressTitleX, pressTitleY);
  tft.textWrite("Pressure");
  // pitch box
  tft.textSetCursor(gyroTitleX, gyroTitleY);
  tft.textWrite("Gyro");
  // yaw box
  tft.textSetCursor(accelTitleX, accelTitleY);
  tft.textWrite("Accel");
  // roll box
  tft.textSetCursor(altTitleX, altTitleY);
  tft.textWrite("Altitude"); 

  tft.textEnlarge(1);

  tft.textSetCursor(xgyroX, xgyroY);
  tft.textWrite("X: ");
  tft.textSetCursor(ygyroX, ygyroY);
  tft.textWrite("Y: ");
  tft.textSetCursor(zgyroX, zgyroY);
  tft.textWrite("Z: ");

  tft.textSetCursor(xaccelX, xaccelY);
  tft.textWrite("X: ");
  tft.textSetCursor(yaccelX, yaccelY);
  tft.textWrite("Y: ");
  tft.textSetCursor(zaccelX, zaccelY);
  tft.textWrite("Z: ");

  tft.textEnlarge(0);
  tft.textSetCursor(signalIndicatorX, signalIndicatorY);
  tft.textWrite("Signal: ");

  // test data structures for display updating
  float gps[4] = {0, 1, 2, 3};
  float imu1[6] = {0, 1, 2, 3, 4, 5};
  float imu2[6] = {0, 1, 2, 3, 4, 5};
  float barom[2] = {0, 1};

  // updateGFXValues(gps, imu1, barom, 0.00, 0);
  endSPI(DISPCS);
  Serial.println("Finished display initialization");

}




// pre-optimization:
// average runtime 50,000 microseconds = 0.05 seconds = 0.13 seconds per loop = max display refresh rate = 7.6 Hz
// post optimization

void loop() {
  
  int size = LoRa.parsePacket();
  if (size) {
    // float startTime = micros();
    // parse in message
    String message = "";
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }

    if (message == "ACK_RECORD") {
      FCRecStatus = "rec";
    }

    if (FCRecStatus == "rec") {
      lastReception = millis();
      // print to Serial and SD card
      dataFile = SD.open("dataGS.txt", FILE_WRITE);
      Serial.println(message);   dataFile.print(message); dataFile.print("|");
      // checksum logic - extracting received checksum and comparing with locally calculated checksum
      int checksumIndex = message.indexOf("/") + 2;

      String checksumStr = message.substring(checksumIndex, message.length() - 1);
      String originalMessage = message.substring(0, checksumIndex - 2);

      uint16_t calculatedChecksum = crc16_ccitt((uint8_t*) originalMessage.c_str(), originalMessage.length());
      String calculatedChecksumStr = String(calculatedChecksum);
      if (checksumStr != calculatedChecksumStr) {
        Serial.println("Checksums do not match");
        dataFile.println("||CORRUPTED||");
        delay(0.1);
      } else {
        Serial.println("Checksums match");
        dataFile.println();
        delay(0.1);
      }
      float data[13]; int prevComma = 0; int dataIndex = 0;
      for (int i = 0; i < originalMessage.length(); i++) {
        if (originalMessage.charAt(i) == ',') {
          String dataStr = originalMessage.substring(prevComma, i);
          data[dataIndex] = dataStr.toFloat();
          prevComma = i + 1;
          dataIndex++;
        }
      }
      // update the display with the new data
      gps[0] = data[3];gps[1] = data[4];gps[2] = data[5];gps[3] = data[6];
      imu[0] = data[7];imu[1] = data[8];imu[2] = data[9];imu[3] = data[10];imu[4] = data[11];imu[5] = data[12];
      barom[0] = data[1];barom[1] = data[2];
      signed int signalStrength = LoRa.packetRssi();
      startSPI(DISPCS, settingsDISP);
      // Serial.print("Receiving message took "); Serial.print(micros() - startTime); Serial.println(" microseconds");
      updateGFXValues(gps, imu, barom, data[0], signalStrength);
      endSPI(DISPCS);
    }
  }
  std::pair<int, int> circleLoc = {110, 220};

  if (millis() - lastReception > 3000) {
    tft.fillCircle(circleLoc.first, circleLoc.second, 15, RA8875_RED);
  } else if (millis() - lastReception > 1000) {
    tft.fillCircle(circleLoc.first, circleLoc.second, 15, RA8875_YELLOW);
  } else {
    tft.fillCircle(circleLoc.first, circleLoc.second, 15, RA8875_GREEN);
  }

  std::pair<int, int> FCStatusLoc = {550, 2};
  tft.textEnlarge(1);
  tft.textSetCursor(FCStatusLoc.first, FCStatusLoc.second);
  if (FCRecStatus == "rec") {
    tft.textColor(RA8875_GREEN, RA8875_BLACK);
    tft.textWrite("FC ack ... rec");
  } else if (FCRecStatus == "wait") {
    tft.textColor(RA8875_YELLOW, RA8875_BLACK);
    tft.textWrite("Wait FC ...");
  } else {
    tft.textColor(RA8875_RED, RA8875_BLACK);
    tft.textWrite("Wait INT");
  }
  tft.textColor(RA8875_WHITE, RA8875_BLACK);
}


// average run time about 80,000 microseconds = 0.08 seconds
void updateGFXValues(float gps[4], float imu[6], float barom[2], float flightTime, signed int signalStrength) {

  // float startGFXTime = micros();

  String lat, lon, alt, speed, xa, ya, za, xg, yg, zg, pressure, temperature;
  lat = String(gps[0], 2); lon = String(gps[1], 2); 
  int latDeg = (int) gps[0]/100;
  float latMin = gps[0] - latDeg*100;
  lat = String(latDeg + latMin/60, 3);
  int lonDeg = (int) gps[1]/100;
  float lonMin = gps[1] - lonDeg*100;
  lon = String(lonDeg + lonMin/60, 3);
  
  alt = String(gps[2], 2); speed = String(gps[3], 2);
  xa = String(imu[0], 2); ya = String(imu[1], 2); za = String(imu[2], 2); xg = String(imu[3], 2); yg = String(imu[4], 2); zg = String(imu[5], 2);
  pressure = String(barom[0], 2); temperature = String(barom[1], 2);

  char latBuf[lat.length() + 1]; char lonBuf[lon.length() + 1]; char altBuf[alt.length() + 1]; char speedBuf[speed.length() + 1];
  char xaBuf[xa.length() + 1]; char yaBuf[ya.length() + 1]; char zaBuf[za.length() + 1]; char xgBuf[xg.length() + 1]; char ygBuf[yg.length() + 1]; char zgBuf[zg.length() + 1];
  char pressureBuf[pressure.length() + 1]; char temperatureBuf[temperature.length() + 1];
  
  lat.toCharArray(latBuf, lat.length() + 1); lon.toCharArray(lonBuf, lon.length() + 1); alt.toCharArray(altBuf, alt.length() + 1); speed.toCharArray(speedBuf, speed.length() + 1);
  xa.toCharArray(xaBuf, xa.length() + 1); ya.toCharArray(yaBuf, ya.length() + 1); za.toCharArray(zaBuf, za.length() + 1); xg.toCharArray(xgBuf, xg.length() + 1); yg.toCharArray(ygBuf, yg.length() + 1); zg.toCharArray(zgBuf, zg.length() + 1);
  pressure.toCharArray(pressureBuf, pressure.length() + 1); temperature.toCharArray(temperatureBuf, temperature.length() + 1);

  // -------------------------------------------------------------------------------------
  tft.textEnlarge(0);
  std::pair<int, int> latPos = {480, 100};
  std::pair<int, int> lonPos = {480, 150};
  std::pair<int, int> altPos = {655, 340};
  std::pair<int, int> speedPos = {268, 120};
  tft.textEnlarge(1);
  tft.textSetCursor(latPos.first, latPos.second);
  tft.textColor(RA8875_WHITE, RA8875_BLACK);
  tft.textWrite(latBuf);
  tft.textSetCursor(lonPos.first, lonPos.second);
  tft.textWrite(lonBuf);
  tft.textEnlarge(1);
  tft.textSetCursor(altPos.first, altPos.second);
  tft.textWrite(altBuf);
  tft.textEnlarge(1);
  tft.textSetCursor(speedPos.first, speedPos.second);
  tft.textWrite(speedBuf);
  tft.textEnlarge(0);

  std::pair<int, int> xaPos = {447, 320};
  std::pair<int, int> yaPos = {447, 370};
  std::pair<int, int> zaPos = {447, 420};
  std::pair<int, int> xgPos = {250, 320};
  std::pair<int, int> ygPos = {250, 370};
  std::pair<int, int> zgPos = {250, 420};
  tft.textEnlarge(1);
  tft.textSetCursor(xaPos.first, xaPos.second);
  tft.textWrite(xaBuf);
  tft.textSetCursor(yaPos.first, yaPos.second);
  tft.textWrite(yaBuf);
  tft.textSetCursor(zaPos.first, zaPos.second);
  tft.textWrite(zaBuf);
  tft.textSetCursor(xgPos.first, xgPos.second);
  tft.textWrite(xgBuf);
  tft.textSetCursor(ygPos.first, ygPos.second);
  tft.textWrite(ygBuf);
  tft.textSetCursor(zgPos.first, zgPos.second);
  tft.textWrite(zgBuf);

  std::pair<int, int> pressurePos = {50, 340};
  std::pair<int, int> temperaturePos = {650, 120};

  tft.textEnlarge(1);
  tft.textSetCursor(pressurePos.first, pressurePos.second);
  tft.textWrite(pressureBuf);
  tft.textEnlarge(0);
  tft.textSetCursor(pressurePos.first + 35, pressurePos.second + 40);
  tft.textWrite("hPa");
  tft.textEnlarge(1);
  tft.textSetCursor(temperaturePos.first, temperaturePos.second);
  tft.textWrite(temperatureBuf);
  tft.textEnlarge(0);
  tft.textSetCursor(temperaturePos.first + 20, temperaturePos.second + 40);
  tft.textWrite("deg C");
  tft.textEnlarge(0);

  std::pair<int, int> signalStrengthPos = {80, 120};


  tft.textEnlarge(1);
  tft.textSetCursor(signalStrengthPos.first, signalStrengthPos.second);
  if (signalStrength > -120) {
    tft.textColor(RA8875_GREEN, RA8875_BLACK);
  } else if (signalStrength > -160) {
    tft.textColor(RA8875_YELLOW, RA8875_BLACK);
  } else {
    tft.textColor(RA8875_RED, RA8875_BLACK);
  }
  tft.textWrite(String(signalStrength).c_str());
  tft.textEnlarge(0);
  tft.textColor(RA8875_WHITE, RA8875_BLACK);
  tft.textSetCursor(signalStrengthPos.first + 14, signalStrengthPos.second + 40);
  tft.textWrite("dBm");

  float flightTimeSec = flightTime/1000000;
  int hours = (int) flightTimeSec/3600;
  int minutes = (int) (flightTimeSec - hours*3600)/60;
  float seconds = flightTimeSec - hours*3600 - minutes*60;
  char flightTimeBuf[22];
  sprintf(flightTimeBuf, "FLIGHT TIME: %02d:%02d:%05.2f s", hours, minutes, seconds);
  tft.textEnlarge(1);
  tft.textSetCursor(flightTimeTitleX, flightTimeTitleY);
  tft.textWrite(flightTimeBuf);
  tft.textEnlarge(0);

  // Serial.print("updateGFXValues took "); Serial.print(micros() - startGFXTime); Serial.println(" microseconds");
}


// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 75

void bmpDraw(const char *filename, int x, int y) {
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col, xpos, ypos;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == false) {
    Serial.println(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.println(F("File size: "));
    Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    // Serial.print(F("Image Offset: "));
    // Serial.println(bmpImageoffset, DEC);

    // Read DIB header
    Serial.print(F("Header size: "));
    Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);

    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: "));
      Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed
        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        ypos = y;
        for (row=0; row<h; row++) { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;

          if (bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }
          xpos = x;
          for (col=0; col<w; col++) { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if(lcdidx > 0) {
                tft.drawPixels(lcdbuffer, lcdidx, xpos, ypos);
                xpos += lcdidx;
                lcdidx = 0;
              }

              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            lcdbuffer[lcdidx++] = color565(r,g,b);
            if (lcdidx >= sizeof(lcdbuffer) || (xpos - x + lcdidx) >= w) {
              tft.drawPixels(lcdbuffer, lcdidx, xpos, ypos);
              lcdidx = 0;
              xpos += lcdidx;
            }
          } // end pixel
            ypos++;
        } // end scanline

        // Write any remaining data to LCD
        if(lcdidx > 0) {
          tft.drawPixels(lcdbuffer, lcdidx, xpos, ypos);
          xpos += lcdidx;
        }

        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");

      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));

}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

byte decToBcd(byte val){
  // Convert normal decimal numbers to binary coded decimal
  return ( (val/10*16) + (val%10) );
}
