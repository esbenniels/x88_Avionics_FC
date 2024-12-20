/**
 ******************************************************************************
 * @file   DISCO_IOT_DataLogTerminal.ino
 * @author  WI6LABS from AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Arduino test application for the STMicrolectronics STM32 DISCO_IOT
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


// Includes.
#include <LPS22HBSensor.h>

#define SerialPort Serial
#define I2C2_SCL    A3
#define I2C2_SDA    A2

// Components
// TwoWire dev_i2c = Wire1;
LPS22HBSensor PressTemp(&Wire);

// SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);
// dev_spi.begin();

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(40, OUTPUT);
  // pinMode(4, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(9600);

  // Initialize I2C bus.
  Wire.begin();

  // Initlialize components.
  PressTemp.begin();
  PressTemp.Enable();
}

void loop() {

  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // digitalWrite(40, HIGH);
  // digitalWrite(4, LOW);

  // Read pressure.
  float pressure, temperature;
  PressTemp.GetPressure(&pressure);
  PressTemp.GetTemperature(&temperature);

  SerialPort.print("Pres[hPa]: ");
  SerialPort.print(pressure, 4);
  SerialPort.print(" | Temp[C]: ");
  SerialPort.println(temperature, 4);
}
