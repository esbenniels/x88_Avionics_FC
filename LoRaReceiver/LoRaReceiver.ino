#include <SPI.h>
#include <LoRa.h>

// #define MISO 12
// #define MOSI 11
// #define SCK 13
#define RADIOCS 10
// const int DISPCS = 0;

// SPISettings settingsRADIO(2000000, MSBFIRST, SPI_MODE1);
// SPISettings settingsDISP(16000000, LSBFIRST, SPI_MODE3);


// void startSPI(const int CS, SPISettings settings) {
//   digitalWrite(CS, LOW);
//   SPI.beginTransaction(settings);
// }

// void endSPI(const int CS) {
//   digitalWrite(CS, HIGH);
//   SPI.endTransaction();
// }

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // pinMode(RADIOCS, OUTPUT);
  // pinMode(DISPCS, OUTPUT);

  SPI.begin();
  // SPI1.begin();

  Serial.println("LoRa Receiver");

  // startSPI(RADIOCS, settingsRADIO);
  // LoRa.setPins(RADIOCS);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // endSPI(RADIOCS);
}

void loop() {
  // try to parse packet
  // startSPI(RADIOCS, settingsRADIO);
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }

  // endSPI(RADIOCS);
}
