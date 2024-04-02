#include <SPI.h>
#include <LoRa.h>
#include <SD.h>

int counter = 0;

int cs = 32;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  LoRa.setPins(cs);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }


  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }

  // Create a new file on the SD card
  File dataFile;
  dataFile = SD.open("dataSender.txt", FILE_WRITE);

  dataFile.println(4);
  dataFile.println(5);
  dataFile.println(2002);
  dataFile.println(10);
  dataFile.println(13);
  dataFile.println(2003);
  dataFile.println(5);
  dataFile.println(10);
  dataFile.println(2003);
  dataFile.println(7);
  dataFile.println(40);
  dataFile.println(10101);

  dataFile.close();

}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);
}
