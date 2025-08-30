// DebugSerial - Lightweight debug Serial library

/*
 * MIT License
 *
 * Copyright (c) 2025 Hiroaki SHIBUKI a.k.a. hidori
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

#ifndef __DEBUG_SERIAL_H__
#define __DEBUG_SERIAL_H__

#include <Arduino.h>

#include "MicroSerial.h"

#if defined(DEBUG)
#define DEBUG_SERIAL_HANDLE(tx, baudrate) MICRO_SERIAL_HANDLE((tx), (baudrate))
#define DEBUG_SERIAL_BEGIN() MicroSerial_begin(__DEBUG_SERIAL_HANDLE__)
#define DEBUG_SERIAL_WAIT_FOR() delay(100)
#define DEBUG_SERIAL_NEWLINE() MicroSerial_newline(__DEBUG_SERIAL_HANDLE__)
#define DEBUG_SERIAL_PRINT(x) MicroSerial_print(__DEBUG_SERIAL_HANDLE__, (x))
#define DEBUG_SERIAL_PRINTLN(x) MicroSerial_println(__DEBUG_SERIAL_HANDLE__, (x))
#define DEBUG_SERIAL_PRINTDEC(x) MicroSerial_printdec(__DEBUG_SERIAL_HANDLE__, (x))
#define DEBUG_SERIAL_PRINTDECLN(x) MicroSerial_printdecln(__DEBUG_SERIAL_HANDLE__, (x))
#define DEBUG_SERIAL_PRINTHEX(x, n) MicroSerial_printhex(__DEBUG_SERIAL_HANDLE__, (x), (n))
#define DEBUG_SERIAL_PRINTHEXLN(x, n) MicroSerial_printhexln(__DEBUG_SERIAL_HANDLE__, (x), (n))
#define DEBUG_SERIAL_PRINTHEX2(x) MicroSerial_printhex(__DEBUG_SERIAL_HANDLE__, (x), 2)
#define DEBUG_SERIAL_PRINTHEX2LN(x) MicroSerial_printhexln(__DEBUG_SERIAL_HANDLE__, (x), 2)
#define DEBUG_SERIAL_PRINTHEX4(x) MicroSerial_printhex(__DEBUG_SERIAL_HANDLE__, (x), 4)
#define DEBUG_SERIAL_PRINTHEX4LN(x) MicroSerial_printhexln(__DEBUG_SERIAL_HANDLE__, (x), 4)
#else
#define DEBUG_SERIAL_HANDLE(tx, baudrate)
#define DEBUG_SERIAL_BEGIN(x)
#define DEBUG_SERIAL_WAIT_FOR()
#define DEBUG_SERIAL_NEWLINE()
#define DEBUG_SERIAL_PRINT(x)
#define DEBUG_SERIAL_PRINTLN(x)
#define DEBUG_SERIAL_PRINTDEC(x)
#define DEBUG_SERIAL_PRINTDECLN(x)
#define DEBUG_SERIAL_PRINTHEX(x, n)
#define DEBUG_SERIAL_PRINTHEXLN(x, n)
#define DEBUG_SERIAL_PRINTHEX2(x)
#define DEBUG_SERIAL_PRINTHEX2LN(x)
#define DEBUG_SERIAL_PRINTHEX4(x)
#define DEBUG_SERIAL_PRINTHEX4LN(x)
#endif

#endif  // __DEBUG_SERIAL_H__
