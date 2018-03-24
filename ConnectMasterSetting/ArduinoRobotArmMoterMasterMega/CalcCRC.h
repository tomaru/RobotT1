#ifndef CalcCRC_h
#define CalcCRC_h

#include "Arduino.h"

unsigned short crc16(unsigned short crc, unsigned char *ptr, int len);

#endif

