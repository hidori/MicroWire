// SCAN - I2C Device Scanner Example

#include <Arduino.h>

#include "DebugSerial.h"
#include "MicroI2C.h"

#if defined(__AVR__)
#define TX PB0
#define SDA PB2
#define SCL PB3
#else
#define TX 8
#define SDA 10
#define SCL 11
#endif

#define __DEBUG_SERIAL_HANDLE__ DEBUG_SERIAL_HANDLE(TX, 9600)
#define I2C MICRO_I2C_HANDLE(SDA, SCL)

uint8_t data[16];

void setup() {
  DEBUG_SERIAL_BEGIN();
  DEBUG_SERIAL_WAIT_FOR();
  DEBUG_SERIAL_NEWLINE();
  DEBUG_SERIAL_PRINTLN("--");

  MicroI2C_begin(I2C);
  delay(100);

  DEBUG_SERIAL_PRINTLN("SCAN");
}

void loop() {
  DEBUG_SERIAL_PRINTLN("--");

  for (uint8_t address = 1; address < 127; address++) {
    if (MicroI2C_beginTransmission(I2C, address)) {
      DEBUG_SERIAL_PRINTHEX(address, 2);
      DEBUG_SERIAL_PRINTLN(" ");
      MicroI2C_endTransmission(I2C);
    }
    delay(10);
  }

  DEBUG_SERIAL_NEWLINE();
  delay(3 * 1000);
}
