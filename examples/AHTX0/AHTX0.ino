// AHTX0 - Temperature and Humidity Sensor Example

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

#define AHTX0_I2C_ADDRESS 0x38
#define AHTX0_CMD_SOFTRESET 0xBA
#define AHTX0_CMD_CALIBRATE 0xBE
#define AHTX0_CMD_TRIGGER 0xAC

uint8_t data[8];

void setup() {
  DEBUG_SERIAL_BEGIN();
  DEBUG_SERIAL_WAIT_FOR();
  DEBUG_SERIAL_NEWLINE();
  DEBUG_SERIAL_PRINTLN("--");

  MicroI2C_begin(I2C);
  delay(100);

  ahtX0_write_cmd(AHTX0_CMD_SOFTRESET, 0x00, 0x00);
  delay(20);
  ahtX0_write_cmd(AHTX0_CMD_CALIBRATE, 0x08, 0x00);
  delay(10);

  DEBUG_SERIAL_PRINTLN("AHTX0");
}

void loop() {
  ahtX0_write_cmd(AHTX0_CMD_TRIGGER, 0x33, 0x00);
  delay(80);

  if (!ahtX0_read_data(data)) {
    DEBUG_SERIAL_PRINTLN("ERROR");
    goto EOL;
  }

  int16_t temperature_x10, humidity_x10;
  ahtX0_calc_temp_hum(data, &temperature_x10, &humidity_x10);
  DEBUG_SERIAL_PRINTDEC(temperature_x10 / 10);
  DEBUG_SERIAL_PRINT(" ");
  DEBUG_SERIAL_PRINTDEC(humidity_x10 / 10);
EOL:
  delay(3 * 1000);
}

void ahtX0_write_cmd(uint8_t cmd, uint8_t data1, uint8_t data2) {
  MicroI2C_beginTransmission(I2C, AHTX0_I2C_ADDRESS);
  MicroI2C_write(I2C, cmd);
  MicroI2C_write(I2C, data1);
  MicroI2C_write(I2C, data2);
  MicroI2C_endTransmission(I2C);
}

bool ahtX0_read_data(uint8_t data[6]) {
  if (!MicroI2C_requestFrom(I2C, AHTX0_I2C_ADDRESS, 6)) {
    DEBUG_SERIAL_PRINTLN("ERROR");
    return false;
  }

  for (uint8_t i = 0; i < 6; i++) {
    MicroI2C_read(I2C, (i < 5) ? true : false, &data[i]);
  }

  return ((data[0] & 0x80) == 0) && !(data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF);
}

void ahtX0_calc_temp_hum(uint8_t data[6], int16_t* temperature_x10, int16_t* humidity_x10) {
  uint32_t raw_temp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
  *temperature_x10 = (int16_t)((raw_temp * 2000UL / 1048576UL) - 500);

  uint32_t raw_hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
  *humidity_x10 = (int16_t)(raw_hum * 1000UL / 1048576UL);
}
