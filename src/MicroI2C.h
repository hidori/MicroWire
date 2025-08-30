// MicroI2C - Lightweight bit-bang I2C Master library

/*
 * MIT License
 *
 * Copyright (c) 2025 Hiroaki SHIBUKI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __MICRO_I2C_H__
#define __MICRO_I2C_H__

#include <stdint.h>
#include <stdbool.h>

// I2C bus definition
#define MICRO_I2C_HANDLE(sda, scl) ((uint8_t)(((0x0f & (uint8_t)(sda)) << 4) | (0x0f & (uint8_t)(scl))))

// High-level APIs

// Initialize I2C pins
void MicroI2C_begin(uint8_t handle);

// Begin I2C transmission to address, returns true if ACK received
bool MicroI2C_beginTransmission(uint8_t handle, uint8_t address);

// End I2C transmission
void MicroI2C_endTransmission(uint8_t handle);

// Request specified quantity of bytes from I2C device, returns true if ACK received
bool MicroI2C_requestFrom(uint8_t handle, uint8_t address, uint8_t quantity);

// Read data byte from I2C device, ack=true sends ACK, ack=false sends NACK
void MicroI2C_read(uint8_t handle, bool ack, uint8_t* data);

// Write data byte, returns true if ACK received
bool MicroI2C_write(uint8_t handle, uint8_t data);

// Middle-level APIs

// Generate I2C start condition
void MicroI2C_start(uint8_t handle);

// Generate I2C stop condition
void MicroI2C_stop(uint8_t handle);

// Low-level APIs

// Set I2C SDA pin high
void MicroI2C_sda_high(uint8_t sda);

// Set I2C SDA pin low
void MicroI2C_sda_low(uint8_t sda);

// Read I2C SDA pin state
bool MicroI2C_sda_read(uint8_t sda);

// Set I2C SCL pin high
void MicroI2C_scl_high(uint8_t scl);

// Set I2C SCL pin low
void MicroI2C_scl_low(uint8_t scl);

// Read I2C SCL pin state
bool MicroI2C_scl_read(uint8_t scl);

// Set I2C SCL pin high and wait for clock stretching
void MicroI2C_scl_high_wait(uint8_t scl);

#endif  // __MICRO_I2C_H__
