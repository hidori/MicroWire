#ifndef __FONT_H__
#define __FONT_H__

#if defined(__AVR__)
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif

static const uint8_t font5x8[] PROGMEM = {
  // ! (0x21)
  0x00,
  0x00,
  0x5F,
  0x00,
  0x00,
  // E (0x45)
  0x7F,
  0x49,
  0x49,
  0x49,
  0x41,
  // H (0x48)
  0x7F,
  0x08,
  0x08,
  0x08,
  0x7F,
  // L (0x4C)
  0x7F,
  0x40,
  0x40,
  0x40,
  0x40,
  // O (0x4F)
  0x3E,
  0x41,
  0x41,
  0x41,
  0x3E,
};

static inline char getChar(const char* str, uint8_t index) {
  return pgm_read_byte(str + index);
}

static inline uint8_t getCharIndex(char c) {
  if (c == '!') return 0;
  if (c == 'E') return 1;
  if (c == 'H') return 2;
  if (c == 'L') return 3;
  if (c == 'O') return 4;
  return 0;
}

#endif
