#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SPI.h>  // something
#include <SD.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"
#include <cstdio>
#include <LoRa.h>


// change these values except the frequency
#define RADIOMISO 12
#define RADIOMOSI 11
#define RADIOSCK 13
#define RADIOCS 10      

#define BUZZ 2

// Connect SCLK to UNO Digital #13 (Hardware SPI clock)
// Connect MISO to UNO Digital #12 (Hardware SPI MISO)
// Connect MOSI to UNO Digital #11 (Hardware SPI MOSI)
#define RA8875_INT 41
#define RA8875_CS 9
#define RA8875_RESET 40

Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);

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
int flightTimeX = 220;
int flightTimeY = 2;
int signalTitleX = 47;
int signalTitleY = 42;
int speedTitleX = 268;
int speedTitleY = 42;
int posTitleX = 460;
int posTitleY = 42;
int latTitleX = 460;
int latTitleY = 100;
int lonTitleX = 460;
int lonTitleY = 130;
int altTitleX = 460;
int altTitleY = 160;
int tempTitleX = 648;
int tempTitleY = 42;
int pressTitleX = 70;
int pressTitleY = 262;
int pitchTitleX = 280;
int pitchTitleY = 262;
int yawTitleX = 480;
int yawTitleY = 262;
int rollTitleX = 675;
int rollTitleY = 262;

File dataFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  // pinMode(BUZZ, OUTPUT);

  LoRa.setPins(RADIOCS);

  // radio init
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    // while(1);
  }

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }


  dataFile = SD.open("dataGS.txt", FILE_WRITE);
  dataFile.print("transmissionTime,signalStrength,latitude,longitude,altitude,speed,temperature,pressure,pitch,yaw,roll//checksum");

  /* Initialize the display using 'RA8875_480x80', 'RA8875_480x128', 'RA8875_480x272' or 'RA8875_800x480' */
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 Not Found!");
    // while (1);
  }

  Serial.println("Found RA8875");

  tft.displayOn(true);
  tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  tft.PWM1out(255);

  // With hardware accelleration this is instant
  tft.fillScreen(RA8875_WHITE);

  Serial.print("(");
  Serial.print(tft.width());
  Serial.print(", ");
  Serial.print(tft.height());
  Serial.println(")");
  tft.graphicsMode();                 // go back to graphics mode
  tft.fillScreen(RA8875_BLACK);
  tft.graphicsMode();
  bmpDraw("MRPL_MissionPatch_24.bmp", 0, 0);

  delay(2000);

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
  tft.cursorBlink(32);
  // Serial.println("Text mode initiated");
  tft.textSetCursor(flightTimeX, flightTimeY);
  // Serial.println("Cursor positioned");
  tft.textEnlarge(1);
  // char string[22] = "FLIGHT TIME: 04:18.50 s";
  tft.textTransparent(RA8875_WHITE);
  tft.textWrite("FLIGHT TIME: 04:18.50 s");
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
  tft.textSetCursor(latTitleX, latTitleY);
  tft.textWrite("Lat: ");
  tft.textSetCursor(lonTitleX, lonTitleY);
  tft.textWrite("Lon: ");
  tft.textSetCursor(altTitleX, altTitleY);
  tft.textWrite("Altitude: ");
  // temperature box
  tft.textSetCursor(tempTitleX, tempTitleY);
  tft.textWrite("Temperature");
  // pressure box
  tft.textSetCursor(pressTitleX, pressTitleY);
  tft.textWrite("Pressure");
  // pitch box
  tft.textSetCursor(pitchTitleX, pitchTitleY);
  tft.textWrite("Pitch");
  // yaw box
  tft.textSetCursor(yawTitleX, yawTitleY);
  tft.textWrite("Yaw");
  // roll box
  tft.textSetCursor(rollTitleX, rollTitleY);
  tft.textWrite("Roll");

  // test data structures for display updating
  float gps[4][10] = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}
  };
  float imu1[6][10] = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}
  };
  float imu2[6][10] = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}
  };
  float barom[2][10] = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}
  };

  updateGFXValues(gps, imu1, imu2, barom);

}



void loop() {
  int size = LoRa.parsePacket();
  if (size) {
    // parse in message
    String message = "";
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }
    // print to Serial and SD card
    Serial.println(message);    dataFile.print(message);
    // checksum logic - extracting received checksum and comparing with locally calculated checksum
    int checksumIndex = message.indexOf("//") + 2;
    Serial.println("Checksum index: " + checksumIndex);
    String checksumStr = message.substring(checksumIndex);
    uint16_t receivedChecksum = (uint16_t) strtol(checksumStr.c_str(), NULL, 16);
    Serial.println("Received checksum: " + receivedChecksum);
    String originalMessage = message.substring(0, checksumIndex - 2);
    // calculate checksum of original message
    uint16_t calculatedChecksum = crc16_ccitt((uint8_t*) originalMessage.c_str(), originalMessage.length());
    Serial.println("Calculated checksum: " + calculatedChecksum);
    //compare calculated checksum with received checksum
    if (receivedChecksum != calculatedChecksum) {
      Serial.println("Checksums do not match");
      dataFile.println("||CORRUPTED||");
    } else {
      Serial.println("Checksums match");
      dataFile.println();
    }
    // split originalMessage into an array of floats
    int numCommas = 0;
    for (int i = 0; i < originalMessage.length(); i++) {
      if (originalMessage.charAt(i) == ',') {
        numCommas++;
      }
    }
    float data[numCommas + 1];
    int prevComma = 0;
    int dataIndex = 0;
    for (int i = 0; i < originalMessage.length(); i++) {
      if (originalMessage.charAt(i) == ',') {
        String dataStr = originalMessage.substring(prevComma, i);
        data[dataIndex] = dataStr.toFloat();
        prevComma = i + 1;
        dataIndex++;
      }
    }
    // update the display with the new data
    float gps[4] = {data[3], data[4], data[5], data[6]};
    float imu[6] = {data[7], data[8], data[9], data[10], data[11], data[12]};
    float barom[2] = {data[1], data[2]};
    updateGFXValues(gps, imu, barom);
  }
}

void updateGFXValues(float gps[][10], float imu1[][10], float imu2[][10], float barom[][10]) {
  float altAvg, latAvg, lonAvg, speedAvg, tempAvg, pressAvg, pitchAvg, yawAvg, rollAvg;
  // iterating through GPS data to obtain latAvg, lonAvg, speedAvg, and altAvg
  float latSum, lonSum, altSum, speedSum = 0;
  for (int i = 0; i < sizeof(gps[0])/4; i++) {
    latSum += gps[0][i]; lonSum += gps[1][i]; altSum += gps[2][i]; speedSum += gps[3][i];
  }
  // Serial.println("Calculated sums");
  // Serial.println(sizeof(gps[0])/4);
  latAvg = latSum / sizeof(gps[0]) * 4; lonAvg = lonSum / sizeof(gps[1]) * 4; altAvg = altSum / sizeof(gps[2]) * 4; speedAvg = speedSum / sizeof(gps[3]) * 4;
  // Serial.print("Calculated averages: "); Serial.print(latAvg); Serial.print(", "); Serial.print(lonAvg); Serial.print(", "); Serial.println(speedAvg);
  int gps_xpos = 510;
  char latAvgStr[20]; char lonAvgStr[20]; char altAvgStr[20]; char speedAvgStr[20];
  dtostrf(latAvg, 5, 2, latAvgStr); dtostrf(lonAvg, 5, 2, lonAvgStr); dtostrf(altAvg, 5, 2, altAvgStr); dtostrf(speedAvg, 5, 2, speedAvgStr);
  tft.textSetCursor(gps_xpos, 100);
  tft.textWrite(latAvgStr);
  tft.textSetCursor(gps_xpos, 130);
  tft.textWrite(lonAvgStr);
  tft.textSetCursor(268, 100);
  tft.textEnlarge(2);
  tft.textWrite(speedAvgStr);
}

void updateGFXValues(float gps[4], float imu[6], float barom[2]) {

  /*
  REQUIRES: gps, imu, and barom are arrays of floats with 4, 6, and 2 elements respectively
  MODIFIES: the display on the TFT screen
  EFFECTS: updates the display with the new values of the flight parameters

  float[] gps = [lat, lon, alt, speed]
  float[] imu = [xa, ya, za, xg, yg, zg]
  float[] barom = [pressure, temperature]
  */

  String lat, lon, alt, speed, xa, ya, za, xg, yg, zg, pressure, temperature;
  lat = String(gps[0], 2); lon = String(gps[1], 2); alt = String(gps[2], 2); speed = String(gps[3], 2);
  xa = String(imu[0], 2); ya = String(imu[1], 2); za = String(imu[2], 2); xg = String(imu[3], 2); yg = String(imu[4], 2); zg = String(imu[5], 2);
  pressure = String(barom[0], 2); temperature = String(barom[1], 2);

  char latBuf[lat.length() + 1]; char lonBuf[lon.length() + 1]; char altBuf[alt.length() + 1]; char speedBuf[speed.length() + 1];
  char xaBuf[xa.length() + 1]; char yaBuf[ya.length() + 1]; char zaBuf[za.length() + 1]; char xgBuf[xg.length() + 1]; char ygBuf[yg.length() + 1]; char zgBuf[zg.length() + 1];
  char pressureBuf[pressure.length() + 1]; char temperatureBuf[temperature.length() + 1];
  
  // convert all the strings to buffers for the textWrite function
  lat.toCharArray(latBuf, lat.length() + 1); lon.toCharArray(lonBuf, lon.length() + 1); alt.toCharArray(altBuf, alt.length() + 1); speed.toCharArray(speedBuf, speed.length() + 1);
  xa.toCharArray(xaBuf, xa.length() + 1); ya.toCharArray(yaBuf, ya.length() + 1); za.toCharArray(zaBuf, za.length() + 1); xg.toCharArray(xgBuf, xg.length() + 1); yg.toCharArray(ygBuf, yg.length() + 1); zg.toCharArray(zgBuf, zg.length() + 1);
  pressure.toCharArray(pressureBuf, pressure.length() + 1); temperature.toCharArray(temperatureBuf, temperature.length() + 1);

  // -------------------------------------------------------------------------------------
  std::pair<int, int> latPos = {510, 100};
  std::pair<int, int> lonPos = {510, 130};
  std::pair<int, int> altPos = {510, 160};
  std::pair<int, int> speedPos = {510, 190};
  // display the data
  tft.textSetCursor(latPos.first, latPos.second);
  tft.textWrite(latBuf);
  tft.textSetCursor(lonPos.first, lonPos.second);
  tft.textWrite(lonBuf);
  tft.textSetCursor(altPos.first, altPos.second);
  tft.textWrite(altBuf);
  tft.textSetCursor(speedPos.first, speedPos.second);
  tft.textWrite(speedBuf);
  // -------------------------------------------------------------------------------------

  std::pair<int, int> xaPos = {510, 220};
  std::pair<int, int> yaPos = {510, 250};
  std::pair<int, int> zaPos = {510, 280};
  std::pair<int, int> xgPos = {510, 310};
  std::pair<int, int> ygPos = {510, 340};
  std::pair<int, int> zgPos = {510, 370};
  // display the data
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
  // -------------------------------------------------------------------------------------
  std::pair<int, int> pressurePos = {510, 400};
  std::pair<int, int> temperaturePos = {510, 430};
  // display the data
  tft.textSetCursor(pressurePos.first, pressurePos.second);
  tft.textWrite(pressureBuf);
  tft.textSetCursor(temperaturePos.first, temperaturePos.second);
  tft.textWrite(temperatureBuf);
  // -------------------------------------------------------------------------------------
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
    // Serial.print(F("Header size: "));
    Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);

    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      // Serial.print(F("Bit Depth: "));
      // Serial.println(bmpDepth);
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


















