#define LED_R 41
#define LED_G 40
#define LED_B 39
#define BUZZ 2

#define GPSRx 28
#define GPSTx 29

void setup() {
  // put your setup code here, to run once:
  // stuff to set up:
    // RADIO - HopeRF RFM95W
      // 0-> CSI
      // 1-> MISO1
      // A13 - 27 -> SCK1
      // A12 - 26 -> MOSI1
    // GPS - Adafruit Ultimate GPS - library downloaded
      // 28 -> Rx
      // 29 -> Tx
    // IMUs - lsm6dsox
      // 13 -> CLK0
      // CS3 - 37 -> CS01
      // CS2 - 36 -> CS02
      // MISO - 12 -> MISO0
      // MOSI - 11 -> MOSI0
    // IMU1 - lsm6dsox
      // OUT1C - 9 -> INT2
      // CS1 - 10 -> INT1
    // IMU2 - lsm6dsox
      // A11 - 25 -> INT1
      // A10 - 24 -> INT2
    // BAROM - lps22hb
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

}

void loop() {
  // put your main code here, to run repeatedly:

}
