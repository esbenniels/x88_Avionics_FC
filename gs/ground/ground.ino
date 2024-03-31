#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <RH_RF95.h>  // radio
#include <SPI.h>  // something
#include <SD.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"
#include <cstdio>


// change these values except the frequency
#define RadioCS 10      
#define RadioRST 9
#define RadioINT 2
#define RadioFREQ 915.0

#define BUZZ 2

RH_RF95 radio(RadioCS, RadioINT);

// Connect SCLK to UNO Digital #13 (Hardware SPI clock)
// Connect MISO to UNO Digital #12 (Hardware SPI MISO)
// Connect MOSI to UNO Digital #11 (Hardware SPI MOSI)
#define RA8875_INT 32
#define RA8875_CS 37
#define RA8875_RESET 35

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  Serial.println("hello");


  pinMode(BUZZ, OUTPUT);

  // radio init
  if (!radio.init()) {Serial.println("Radio init failed");}
  if (!radio.setFrequency(RadioFREQ)) {Serial.println("Radio frequency setting failed");}

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }

  File dataFile;
  dataFile = SD.open("dataGS.txt", FILE_WRITE);

  /* Initialize the display using 'RA8875_480x80', 'RA8875_480x128', 'RA8875_480x272' or 'RA8875_800x480' */
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 Not Found!");
    while (1);
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

  std::pair<int, int> flightTimeLocation = {220, 2};
  std::pair<int, int> 


  tft.textMode();
  tft.cursorBlink(32);
  // Serial.println("Text mode initiated");
  tft.textSetCursor(220, 2);
  // Serial.println("Cursor positioned");
  tft.textEnlarge(1);
  // char string[22] = "FLIGHT TIME: 04:18.50 s";
  tft.textTransparent(RA8875_WHITE);
  tft.textWrite("FLIGHT TIME: 04:18.50 s");
  // signal strength box
  tft.textSetCursor(47, 42);
  tft.textEnlarge(0);
  tft.textWrite("Signal Strength");
  // altitude box
  tft.textSetCursor(268, 42);
  tft.textWrite("Speed");
  // GPS box
  tft.textSetCursor(460, 42);
  tft.textWrite("Position");
  tft.textSetCursor(460, 100);
  tft.textWrite("Lat: ");
  tft.textSetCursor(460, 130);
  tft.textWrite("Lon: ");
  tft.textSetCursor(460, 160);
  tft.textWrite("Altitude: ");
  // temperature box
  tft.textSetCursor(648, 42);
  tft.textWrite("Temperature");
  // pressure box
  tft.textSetCursor(70, 262);
  tft.textWrite("Pressure");
  // pitch box
  tft.textSetCursor(280, 262);
  tft.textWrite("Pitch");
  // yaw box
  tft.textSetCursor(480, 262);
  tft.textWrite("Yaw");
  // roll box
  tft.textSetCursor(675, 262);
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
  // put your main code here, to run repeatedly:
  if (radio.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // receiving message
    if (radio.recv(buf, &len)) {
      if (len > 2) {
        // reconstructing the received CRC using bitwise operations and LSB/MSB shifting
        uint16_t receivedCRC = (buf[len - 2] << 8) | buf[len - 1];  // Reconstruct the CRC
        // calculating the received CRC
        uint8_t calculatedCRC = crc16_ccitt(buf, len-2);

        if (receivedCRC == calculatedCRC) {
          // CRC matches, proceed with writing to SD card
          File dataFile = SD.open("dataGS.txt", FILE_WRITE);
          if (dataFile) {
            // Serial.println(buf);
            // Serial.println();
            dataFile.write(buf, len-2);
            dataFile.close();
            Serial.println("Data written to SD");
          }
          else {
            Serial.println("Error writing to SD");
          }
          // updating display with information
          // updateGFXValues(gps_data, imu1_data, imu2_data, barom_data);
        } else {
          Serial.println("CHECKSUM MISMATCH - POTENTIAL DATA CORRUPTION");
        }
      }

    }
    else {
      Serial.println("Receive failed");
    }
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

















